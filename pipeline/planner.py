"""LLM planner: natural-language prompt -> MissionPlan JSON.

Provider is selected by the LLM_PROVIDER env var:
  - "anthropic" (default): Claude via api.anthropic.com
  - "groq":                Llama via api.groq.com (OpenAI-compatible)
  - "gemini":              Gemini via generativelanguage.googleapis.com

Each provider needs a matching API key env var:
  ANTHROPIC_API_KEY, GROQ_API_KEY, or GEMINI_API_KEY.

The rest of the pipeline doesn't care which provider was used — the LLM's
job is to emit JSON that passes the validator. Failure feedback loop is
identical across providers.
"""
import json
import os
import re
from typing import Optional

import requests

from .schema import MissionPlan, schema_for_prompt
from .validator import validate, ValidationError

PROVIDER = os.environ.get("LLM_PROVIDER", "anthropic").lower()
MAX_RETRIES = 2

SYSTEM_PROMPT = """You are a mission planner for a quadcopter (or ground robot) \
operating in a simulator. Convert the operator's natural-language instruction into \
a single JSON object conforming EXACTLY to this JSON schema:

{schema}

Known named routes (prefer these when the operator references them):
{routes}

Rules:
- Output ONLY the JSON object. No markdown fences, no prose, no explanation.
- Prefer action "patrol_route" with a known route name when the instruction
  matches one; use "fly_waypoints" only for explicit geometry.
- For "follow X" instructions use action "follow_target" with target_class=X.
- For "go to (X, Y)" or "navigate to <point>" on ground robots use action
  "navigate_to" with goal_x_m and goal_y_m.
- Respect any altitude / speed / repetition the operator states. Defaults:
  altitude 10 m, speed 5 m/s, 1 loop.
- Put a one-line justification in "reasoning".
- You only propose plans. You have no direct control of the vehicle.
"""

# --------------------------------------------------------------------------
# Provider-specific transport
# --------------------------------------------------------------------------

def _anthropic_call(system: str, messages: list) -> str:
    resp = requests.post(
        "https://api.anthropic.com/v1/messages",
        headers={
            "x-api-key": os.environ["ANTHROPIC_API_KEY"],
            "anthropic-version": "2023-06-01",
            "content-type": "application/json",
        },
        json={
            "model": os.environ.get("PLANNER_MODEL", "claude-sonnet-4-6"),
            "max_tokens": 1024,
            "system": system,
            "messages": messages,
        },
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["content"][0]["text"]


def _groq_call(system: str, messages: list) -> str:
    # Groq speaks OpenAI's chat-completions dialect
    chat_msgs = [{"role": "system", "content": system}] + messages
    resp = requests.post(
        "https://api.groq.com/openai/v1/chat/completions",
        headers={
            "Authorization": f"Bearer {os.environ['GROQ_API_KEY']}",
            "Content-Type": "application/json",
        },
        json={
            "model": os.environ.get("PLANNER_MODEL", "llama-3.3-70b-versatile"),
            "messages": chat_msgs,
            "max_tokens": 1024,
            "response_format": {"type": "json_object"},   # forces valid JSON
            "temperature": 0.1,
        },
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["choices"][0]["message"]["content"]


def _gemini_call(system: str, messages: list) -> str:
    # Gemini uses a single "contents" list with the system prompt as the first user turn
    model = os.environ.get("PLANNER_MODEL", "gemini-2.0-flash")
    contents = [{"role": "user", "parts": [{"text": system}]}]
    for m in messages:
        role = "user" if m["role"] == "user" else "model"
        contents.append({"role": role, "parts": [{"text": m["content"]}]})
    resp = requests.post(
        f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
        headers={"Content-Type": "application/json"},
        params={"key": os.environ["GEMINI_API_KEY"]},
        json={
            "contents": contents,
            "generationConfig": {
                "responseMimeType": "application/json",
                "maxOutputTokens": 1024,
                "temperature": 0.1,
            },
        },
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["candidates"][0]["content"]["parts"][0]["text"]


_PROVIDER_DISPATCH = {
    "anthropic": _anthropic_call,
    "groq":      _groq_call,
    "gemini":    _gemini_call,
}


def _call_llm(prompt: str, routes: dict, feedback: Optional[dict] = None) -> str:
    route_desc = "\n".join(f"- {k}: {v.get('description', '')}"
                           for k, v in routes.items())
    system = SYSTEM_PROMPT.format(schema=schema_for_prompt(), routes=route_desc)

    messages = [{"role": "user", "content": prompt}]
    if feedback:
        messages.append({"role": "assistant", "content": feedback["previous"]})
        messages.append({"role": "user",
                         "content": f"That plan was REJECTED by the safety validator: "
                                    f"{feedback['errors']}. Emit a corrected JSON object."})

    call = _PROVIDER_DISPATCH.get(PROVIDER)
    if call is None:
        raise SystemExit(f"[planner] unknown LLM_PROVIDER={PROVIDER!r}. "
                         f"Choose one of {sorted(_PROVIDER_DISPATCH)}.")
    return call(system, messages)


def _strip_fences(text: str) -> str:
    return re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.M).strip()


def plan_with_llm(prompt: str, routes: dict) -> MissionPlan:
    """Prompt -> validated MissionPlan, with bounded self-correction retries."""
    print(f"[planner] using provider={PROVIDER}")
    feedback = None
    for attempt in range(1 + MAX_RETRIES):
        raw = _call_llm(prompt, routes, feedback)
        try:
            plan = MissionPlan.model_validate_json(_strip_fences(raw))
            validate(plan, routes)
            return plan
        except (ValidationError, ValueError) as e:
            print(f"[planner] attempt {attempt + 1} rejected: {e}")
            feedback = {"previous": raw, "errors": str(e)}
    raise SystemExit("[planner] LLM could not produce a valid plan — mission rejected.")


# --------------------------------------------------------------------------
# Offline deterministic parser (no keys, no network)
# --------------------------------------------------------------------------
_WORD_NUMBERS = {"once": 1, "twice": 2, "thrice": 3, "three times": 3}


def plan_offline(prompt: str, routes: dict) -> MissionPlan:
    p = prompt.lower()

    alt = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|metre|meter)", p)
    spd = re.search(r"(\d+(?:\.\d+)?)\s*m/s", p)
    dur = re.search(r"(?:for\s+)?(\d+)\s*(?:s|sec|second)", p)

    # navigate_to intent
    if any(w in p for w in ("navigate", "go to", "drive to")):
        m = re.search(r"[\(\[]?\s*(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)", p)
        if m:
            return MissionPlan(
                action="navigate_to",
                goal_x_m=float(m.group(1)),
                goal_y_m=float(m.group(2)),
                reasoning=f"offline parse: navigate_to ({m.group(1)}, {m.group(2)})",
            )

    # follow_target intent
    if any(w in p for w in ("follow", "chase", "track", "tail")):
        m = re.search(r"(?:follow|chase|track|tail)\s+(?:the\s+|any\s+)?(\w+)", p)
        target = m.group(1) if m else "person"
        return MissionPlan(
            action="follow_target",
            target_class=target,
            altitude_m=float(alt.group(1)) if alt else 10.0,
            follow_duration_s=float(dur.group(1)) if dur else 30.0,
            reasoning=f"offline parse: follow_target '{target}'",
        )

    # patrol_route intent (default)
    route = next((r for r in routes if r.split("_")[0] in p or r in p), None)
    if route is None and any(w in p for w in ("sweep", "survey", "lawnmower")):
        route = "survey_lawnmower"
    if route is None and any(w in p for w in ("perimeter", "patrol", "loop")):
        route = "perimeter"
    if route is None and any(w in p for w in ("inspect", "fence")):
        route = "inspection"
    if route is None:
        raise SystemExit(f"[offline planner] no known route or intent matches: {prompt!r}")

    loops = 1
    for word, n in _WORD_NUMBERS.items():
        if word in p:
            loops = n
    m = re.search(r"(\d+)\s*(?:times|loops)", p)
    if m:
        loops = int(m.group(1))

    return MissionPlan(
        action="patrol_route", route=route,
        altitude_m=float(alt.group(1)) if alt else 10.0,
        speed_ms=float(spd.group(1)) if spd else 5.0,
        loops=loops,
        reasoning=f"offline parse: patrol_route '{route}'",
    )