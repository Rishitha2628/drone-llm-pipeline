# Approach & Architecture

## Core pipeline

**Prompt → LLM → validated mission JSON → deterministic executor → PX4 SITL.**

Design decisions and why:

1. **The LLM only proposes.** Its entire output surface is one JSON object
   validated against a Pydantic schema. It cannot touch MAVLink, cannot see
   the safety limits (so it can't argue with them), and has no channel into
   the control loop.
2. **Named routes as the primary vocabulary.** Instead of letting the LLM
   invent arbitrary geometry, it maps intent onto whitelisted routes
   (`perimeter`, `inspection`, `survey_lawnmower`) defined in `routes.yaml`.
   Free-form waypoints are still allowed but pass through the same geofence.
   This shrinks the attack/error surface dramatically.
3. **Two-stage validation.** (a) Pydantic schema: types, enums, bounds,
   structural rules. (b) Safety validator: altitude/speed/loop limits,
   geofence radius, route whitelist. Violations are collected exhaustively
   and fed back to the LLM for a bounded retry (max 2); after that the
   mission is hard-rejected — we never silently repair a plan.
4. **Deterministic, auditable executor.** Pure function of the validated
   JSON: same JSON → same MAVSDK offboard command sequence. Every command is
   timestamped in the log, and every executed mission is persisted to
   `missions/last_mission.json`, which can be replayed byte-for-byte with
   `--mission` (this is also how you diff "what the operator asked" vs "what
   flew").
5. **Offline mode (`--no-llm`).** A deterministic keyword parser proves the
   pipeline end-to-end without network/keys — useful for CI and for the
   examiner.

## Challenge 3 — Vision AI target detection + follow (attempted)

Plan (building on monemati/PX4-ROS2-Gazebo-YOLOv8, AGPL-3.0):

- **Sim**: PX4 + Gazebo Harmonic x500 with a gimballed camera; camera frames
  bridged into ROS 2 via `ros_gz_bridge` on `/camera/image_raw`.
- **Detection node**: Ultralytics YOLOv8n subscribed to the camera topic. The
  target class is **user-configurable**: the mission JSON gains a
  `follow_target` action with a `target_class` field (e.g. "person", "car"),
  validated against YOLO's class list — same guardrail philosophy.
- **On first detection**: save the annotated frame and publish it to the
  operator (file drop + optional webhook/Telegram bot), then switch the
  executor into follow mode.
- **Follow controller** (deterministic, LLM-free): a P-controller on the
  bounding-box centroid error → MAVSDK velocity setpoints
  (`set_velocity_body`): yaw rate from horizontal pixel error, forward speed
  from bounding-box area (distance proxy), altitude hold. Loss-of-target for
  >3 s → hover, >10 s → resume search pattern or RTL.
- **Safety**: follow mode inherits the same geofence — the validator's
  limits are enforced continuously in the executor loop, so the target
  cannot lure the drone out of bounds.

## Challenge 1 — Multi-agent formations (approach overview)

- Launch 2–3 PX4 SITL instances (PX4 supports multi-vehicle Gazebo natively;
  each gets its own MAVLink port 14540/14541/14542).
- Extend the schema with a `squad_mission` type: the LLM emits **squad-level
  intent** (task + formation + area), never per-drone micro-commands.
- A coordination layer (deterministic) decomposes that into per-drone
  waypoint sets: leader-follower with fixed NED offsets for "wedge"/"line"
  (reference: artastier/PX4_Swarm_Controller), or route splitting by arc
  length for "divide this route between you".
- Each drone runs the same single-vehicle executor; a supervisor synchronises
  lap boundaries via simple barrier waits on telemetry.

## Challenge 2 — SLAM / autonomous navigation (approach overview)

- Swap the vehicle layer: TurtleBot3 in Gazebo + Nav2 + slam_toolbox.
- The mission JSON gains `explore`/`navigate_to` actions; the executor
  becomes a Nav2 action client sending `NavigateToPose` goals — the
  LLM-out-of-the-loop property is preserved because Nav2 owns local planning
  and obstacle avoidance.
- For drones: PX4 + a depth camera + RTAB-Map for 3D SLAM, with the executor
  sending position setpoints only inside the mapped free space.

## Scaling to the real world

- **Sim-to-real**: the executor talks MAVSDK/MAVLink, so the same code path
  drives a real PX4 flight controller — swap `udp://:14540` for a serial or
  telemetry link. Gaps to close: wind/GPS noise (tune arrival radius +
  timeouts), failsafes (battery, link-loss → RTL is PX4-native), and a
  pre-flight validator pass against a real geofence polygon, not a radius.
- **Guardrails harden, LLM stays soft**: in production the validator grows
  (no-fly zones from GeoJSON, terrain-aware altitude, ROC/DGCA rule checks)
  while the LLM layer can be swapped or fine-tuned freely — the interface is
  just JSON.
- **Fleet scale**: mission JSONs are queued/persisted (they're already the
  audit artifact), executors become per-vehicle services, and a supervisor
  reconciles desired vs actual state — essentially a control plane pattern.
- **Human-in-the-loop**: because plans are validated *before* execution, a
  production system inserts an operator "confirm" step showing the rendered
  route — cheap because the JSON is the single source of truth.

## Sources

See README §Sources. All external code is cited with repo + license; the
pipeline code in `pipeline/` is original, with the executor's MAVSDK usage
patterned on official MAVSDK-Python examples (BSD-3).
