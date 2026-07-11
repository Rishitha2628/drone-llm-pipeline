"""LLM planner: natural-language prompt -> MissionPlan JSON.

Design rules:
  * The LLM PROPOSES, it never flies. Its only output is JSON.
  * On validation failure the errors are fed back and the LLM gets a bounded
    number of retries. If it still fails, the mission is rejected — we never
    "fix up" an invalid plan silently.
  * --no-llm mode uses a small deterministic keyword parser so the whole
    pipeline can be demonstrated offline / without API keys.
"""
import json
import os
import re
from typing import Optional

import requests

from .schema import MissionPlan, schema_for_prompt
from .validator import validate, ValidationError

ANTHROPIC_URL = "https://api.anthropic.com/v1/messages"
MODEL = os.environ.get("PLANNER_MODEL", "claude-sonnet-4-6")
MAX_RETRIES = 2

SYSTEM_PROMPT = """You are a mission planner for a quadcopter operating in a \
simulator. Convert the operator's natural-language instruction into a single \
JSON object conforming EXACTLY to this JSON schema:

{schema}

Known named routes (prefer these when the operator references them):
{routes}

Rules:
- Output ONLY the JSON object. No markdown fences, no prose.
- Prefer action "patrol_route" with a known route name when the instruction
  matches one; use "fly_waypoints" only for explicit geometry.
- Respect any altitude / speed / repetition the operator states. If not
  stated, use sensible defaults (altitude 10 m, speed 5 m/s, 1 loop).
- Put a one-line justification in "reasoning".
- You only propose plans. You have no direct control of the vehicle.
"""


def _call_llm(prompt: str, routes: dict, feedback: Optional[str] = None) -> str:
    route_desc = "\n".join(f"- {k}: {v['description']}" for k, v in routes.items())
    system = SYSTEM_PROMPT.format(schema=schema_for_prompt(), routes=route_desc)
    messages = [{"role": "user", "content": prompt}]
    if feedback:
        messages.append({"role": "assistant", "content": feedback["previous"]})
        messages.append({"role": "user", "content":
                         f"That plan was REJECTED by the safety validator: "
                         f"{feedback['errors']}. Emit a corrected JSON object."})
    resp = requests.post(
        ANTHROPIC_URL,
        headers={
            "x-api-key": os.environ["ANTHROPIC_API_KEY"],
            "anthropic-version": "2023-06-01",
            "content-type": "application/json",
        },
        json={"model": MODEL, "max_tokens": 1000,
              "system": system, "messages": messages},
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["content"][0]["text"]


def _strip_fences(text: str) -> str:
    return re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.M).strip()


def plan_with_llm(prompt: str, routes: dict) -> MissionPlan:
    """Prompt -> validated MissionPlan, with bounded self-correction retries."""
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


# ---------------------------------------------------------------------------
# Offline fallback: deterministic keyword parser (no network, no keys).
# ---------------------------------------------------------------------------
_WORD_NUMBERS = {"once": 1, "twice": 2, "thrice": 3, "three times": 3}


def plan_offline(prompt: str, routes: dict) -> MissionPlan:
    p = prompt.lower()

    alt = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|metre|meter)", p)
    spd = re.search(r"(\d+(?:\.\d+)?)\s*m/s", p)
    dur = re.search(r"(?:for\s+)?(\d+)\s*(?:s|sec|second)", p)

    # follow_target intent
    follow_words = ("follow", "chase", "track", "tail")
    if any(w in p for w in follow_words):
        # pick target class from a small vocabulary
        classes = ("person", "car", "truck", "bus", "motorcycle",
                   "bicycle", "dog", "cat", "sheep", "cow", "horse")
        # Grab ANY noun-ish word after the follow verb; validator gates it.
 
        m = re.search(r"(?:follow|chase|track|tail)\s+(?:the\s+|any\s+)?(\w+)", p)
        target = m.group(1) if m else "person"
        return MissionPlan(
            action="follow_target",
            target_class=target,
            altitude_m=float(alt.group(1)) if alt else 10.0,
            follow_duration_s=float(dur.group(1)) if dur else 30.0,
            reasoning=f"offline keyword parse matched follow_target for '{target}'",
        )

    # patrol / waypoint intent (original path)
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
        reasoning=f"offline keyword parse matched route '{route}'",
    )
