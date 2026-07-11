# LLM-Commanded Drone Mission Pipeline

Natural-language prompt → LLM → **validated mission JSON** → **deterministic
executor** → PX4 SITL. The LLM proposes; it never flies.

```
 ┌────────┐   ┌─────────┐   ┌────────────────┐   ┌───────────────┐   ┌──────────┐
 │ Prompt │──▶│ Planner │──▶│ Schema + Safety│──▶│ Deterministic │──▶│ PX4 SITL │
 │  (NL)  │   │  (LLM)  │   │   Validator    │   │   Executor    │   │ (Gazebo) │
 └────────┘   └─────────┘   └────────────────┘   └───────────────┘   └──────────┘
                   ▲                │ rejected plans fed back
                   └────────────────┘ (max 2 retries, then hard reject)
```

## Quick start (Docker, recommended)

```bash
docker compose up -d px4-sim          # PX4 SITL + headless Gazebo
export ANTHROPIC_API_KEY=sk-ant-...   # or use --no-llm (offline parser)
docker compose run pipeline "Patrol the perimeter loop twice at 15 metres"
```

## Native install (Ubuntu 22.04/24.04)

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
# PX4 SITL (one-time):
git clone https://github.com/PX4/PX4-Autopilot --recursive
cd PX4-Autopilot && make px4_sitl gz_x500      # terminal 1
# then, in this repo:
python -m pipeline.main "Patrol the perimeter loop twice at 15 metres"
```

## Commands to try

| Prompt | Expected behaviour |
|---|---|
| `"Patrol the perimeter loop twice at 15 metres"` | Square loop ×2 at 15 m, then RTL |
| `"Drive the inspection route and return to start"` | L-shaped route, 10 m default |
| `"Sweep the survey area at 8 m/s"` | Lawnmower pattern at 8 m/s |
| `"Patrol the perimeter at 60 metres"` | **REJECTED** — altitude above 50 m limit |
| `--mission missions/last_mission.json` | Replays an audited mission byte-for-byte |

Useful flags: `--no-llm` (offline deterministic parser, no API key needed),
`--dry-run` (plan + validate only), `--sim udp://:14540` (MAVSDK address).

## Guardrails (validator layer)

Hard limits, invisible to the LLM: altitude 2–50 m, speed 0.5–12 m/s, ≤10
loops, ≤50 waypoints, 200 m geofence around home, only whitelisted actions
and named routes. Every executed mission is written to
`missions/last_mission.json` for audit/replay.

## Sources / citations

- **PX4 Autopilot + SITL** — github.com/PX4/PX4-Autopilot (BSD-3) — flight stack + simulator.
- **MAVSDK-Python** — github.com/mavlink/MAVSDK-Python (BSD-3) — offboard/telemetry API; executor structure adapted from its `offboard_position_ned.py` example.
- **jonasvautherin/px4-gazebo-headless** (BSD-3) — headless PX4 SITL Docker image used in docker-compose.
- **ChatDrones** — github.com/Gaurang-1402/ChatDrones (MIT) — referenced for the NL→JSON command pattern; no code copied.
- Architecture write-up: see `docs/APPROACH.md`.
