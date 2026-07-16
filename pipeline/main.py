"""Entry point: prompt -> LLM -> validated mission JSON -> executor -> PX4 SITL.

Dispatches on plan.action:
  - patrol_route, fly_waypoints, return_home -> waypoint executor (executor.py)
  - follow_target                            -> follow controller (follow_controller.py)
  - squad_patrol                             -> squad executor  (squad_executor.py)
  - navigate_to                              -> nav2 client (executor_nav2.py, TBD)
"""
import argparse
import asyncio
import json
import pathlib
import sys

from .schema import MissionPlan, Action
from .validator import load_routes, validate, ValidationError
from .planner import plan_with_llm, plan_offline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prompt", nargs="?")
    ap.add_argument("--no-llm", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--mission")
    ap.add_argument("--sim", default="udpin://0.0.0.0:14540")
    ap.add_argument("--base-port", type=int, default=14540,
                    help="Base MAVLink port. Squad missions use base, base+1, ...")
    args = ap.parse_args()

    routes = load_routes(str(pathlib.Path(__file__).parent / "routes.yaml"))

    if args.mission:
        plan = MissionPlan.model_validate_json(pathlib.Path(args.mission).read_text())
        print(f"[main] loaded saved mission {args.mission}")
    elif args.prompt:
        print(f"[main] prompt: {args.prompt!r}")
        plan = (plan_offline if args.no_llm else plan_with_llm)(args.prompt, routes)
    else:
        ap.error("provide a prompt or --mission FILE")

    try:
        plan, waypoints = validate(plan, routes)
    except ValidationError as e:
        print("[main] plan REJECTED by safety validator ✗")
        for err in e.errors:
            print(f"        - {err}")
        sys.exit(1)
    print("[main] plan VALIDATED ✓")
    print(json.dumps(json.loads(plan.model_dump_json()), indent=2))

    out = pathlib.Path("missions") / "last_mission.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(plan.model_dump_json(indent=2))
    print(f"[main] audited mission saved to {out}")

    if args.dry_run:
        print("[main] dry run — stopping before execution")
        return

    if plan.action == Action.FOLLOW_TARGET:
        from .follow_controller import follow
        asyncio.run(follow(plan, system_address=args.sim))
    elif plan.action == Action.SQUAD_PATROL:
        from .squad_executor import execute as squad_execute
        asyncio.run(squad_execute(plan, waypoints, base_port=args.base_port))
    else:
        from .executor import execute
        asyncio.run(execute(plan, waypoints, system_address=args.sim))


if __name__ == "__main__":
    sys.exit(main())
