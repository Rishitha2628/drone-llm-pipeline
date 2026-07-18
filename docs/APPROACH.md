# Approach & Architecture

## 1. Core pipeline

**Prompt → LLM planner → schema validation → safety validation → deterministic executor → PX4 SITL.**

```
Operator prompt ("patrol the perimeter twice at 15 metres")
      │
      ▼
 LLM planner (pipeline/planner.py)
      │   proposes a MissionPlan JSON — and nothing else.
      │   Providers: Gemini / Groq / Anthropic, selected by LLM_PROVIDER.
      │   Offline fallback: deterministic keyword parser (--no-llm).
      ▼
 Pydantic schema (pipeline/schema.py)
      │   strict typed contract; unknown actions/fields are impossible.
      ▼
 Safety validator (pipeline/validator.py)
      │   altitude 2–50 m · speed 0.5–12 m/s · 200 m geofence ·
      │   route whitelist · target-class whitelist · squad spacing ≥ 2 m ·
      │   follow duration ≤ 120 s · ≤ 50 waypoints · ≤ 3 drones.
      │   On rejection: errors are fed back to the LLM (max 2 retries),
      │   then the mission is refused. Bounded, auditable self-correction.
      ▼
 Deterministic executor(s)
      │   executor.py (single drone) · squad_executor.py (multi) ·
      │   follow_controller.py (vision follow).
      │   Same JSON in ⇒ same flight out. Every accepted mission is
      │   written to missions/last_mission.json (audit trail — this file
      │   caught a real route-dispatch bug during development).
      ▼
 PX4 SITL + Gazebo Harmonic (MAVSDK offboard, localhost:14540+)
```

**The one design rule everything follows: the LLM is never in the control
loop.** It proposes intent once, before takeoff. Perception, control, and
coordination are deterministic code. This bounds the blast radius of model
error to "a plan the validator must approve" — never to a motor command.

### Design decisions worth defending

- **Why a bounded retry loop (2 attempts)?** Unbounded retries can loop on a
  systematically-wrong model; zero retries wastes the LLM's ability to correct
  format slips. Two retries with explicit error feedback empirically fixes
  most rejections while keeping worst-case latency bounded.
- **Why NED waypoints instead of GPS?** SITL local frame is deterministic and
  reproducible for an examiner; the executor is frame-agnostic and would take
  global positions with a one-line change.
- **Why a route whitelist?** The LLM maps fuzzy language ("the fence line") to
  named, pre-surveyed geometry. It can choose *which* route, never invent one
  — geofence violations become impossible by construction for routed missions.
- **Schema honesty:** MissionPlan is a union across all action types, so a
  patrol mission carries default-valued follow/squad fields. Cleaner design
  would be per-action sub-models (discriminated union). Known, deliberate debt.

## 2. Challenge 3 — vision detect + follow

- **YOLOv8n** on the Gazebo camera topic (30 FPS sustained), native process
  (Gazebo's shared-memory transport is invisible from containers).
- Detector → controller contract is a small shared-state file
  (`/tmp/target_state.json`) — decoupled processes, either restartable alone.
- **Follow controller is a plain P-controller** (yaw from bbox x-error,
  forward speed from bbox area error): fully explainable, tunable, no learned
  policy in the control path. States: SEARCH (slow yaw) → LOCKED → HOVER on
  loss → RTL on timeout.
- Moving target: a Gazebo Actor walking a scripted loop; the drone visibly
  tracks it. Fixed forward-down camera means a directly-overhead target exits
  frame — the controller's hover-then-search response to that is correct
  behaviour; a production airframe would carry a gimbal.
- Every detection saves an annotated JPG (bbox + banner) = the "picture to the
  operator" requirement. Target class is operator/LLM-configurable and
  whitelist-gated ("follow the giraffe" → validator rejection, demo-able).

## 3. Challenge 1 — multi-drone formations

- LLM chooses `formation` (line / side_by_side) and `mode` (mirror = same
  route offset, split = route divided across drones); a deterministic
  decomposition layer turns squad intent into per-drone waypoint lists.
- Two PX4 SITL instances share one Gazebo world (`px4 -i 1`, ports auto-offset
  14540/14541); drone 1 spawns already in formation.
- **Collision safety, three layers:** validator floor on spacing (≥2 m);
  asyncio **barrier at every waypoint** (not just laps — a fixed offset on a
  closed polygon collapses at corners if one drone rounds a corner early);
  per-drone altitude shelves (+2 m/drone).
- **The bug worth retelling:** first flight sent both drones to the *same
  world point*. Each PX4 instance's NED origin is its own spawn point, so the
  follower's +5 m-north spawn exactly cancelled the −5 m-south formation
  offset. Fix: `local_wp = route_wp + formation_offset − home_offset`, and
  spawn in formation so transit paths never cross. Unit tests now assert
  exact spacing at every waypoint *and* minimum separation during transit.

## 4. Challenge 2 — SLAM. What shipped, and the honest journey

**Shipped:** online 2D SLAM with **slam_toolbox** on the `x500_lidar_2d`
airframe in PX4's walls world. PX4 EKF odometry is bridged to ROS
(`slam/odom_bridge.py` → `/odom` + TF), the 2D lidar is bridged to `/scan`,
and a collision-verified coverage route (`walls_survey`, generated by a BFS
planner over the wall geometry and re-verified leg-by-leg) sweeps all four
walls. Result: live occupancy grid in RViz during flight; saved artifact
`docs/evidence/walls_map.pgm` — 201×114→209×184 cells @ 0.1 m, **241 → 598
occupied cells** after the survey flight. Constant-altitude flight makes the
drone a planar robot, which is exactly slam_toolbox's operating assumption.

### The path here — three failed architectures, each abandoned for measured reasons

1. **Depth-camera ICP odometry (rtabmap icp_odometry).** Structurally
   degenerate: on open ground *and* in the walls world, scan "complexity"
   stayed below threshold (ratio pinned at 0.000) — a forward-facing depth cam
   sees too little 3D structure to constrain 6-DoF ICP (eigenvalue-starved
   along at least one axis), producing wild pose guesses. This is why real
   drones use visual-*inertial* odometry rather than pure depth ICP.
2. **Hybrid: PX4 odometry + rtabmap mapping (depth cloud).** Architecturally
   sound (external odom + mapper is the production pattern) and it did map —
   but the raw depth cloud we initially bridged is ~220 MB/s over loopback,
   and under flight load the *entire ROS graph* stalled machine-wide
   (measured: /clock, /odom and cloud all freezing together for tens of
   seconds, then resuming in sync). One accepted scan per ~7 m of travel
   cannot sustain scan matching.
3. **Pivot: 2D lidar.** A LaserScan is a few KB vs megabytes per frame —
   the bandwidth failure mode disappears by construction, and slam_toolbox
   is the canonical, battle-tested consumer.

### Issues faced on the final stack (all diagnosed to root cause)

- **Sim-time application is unreliable on Humble:** `use_sim_time` via
  params-file or post-hoc set silently failed for some nodes → wall-clock
  stamps → "message earlier than transform cache" drops. Rule: pass
  `-p use_sim_time:=true` on every node's command line (encoded in
  `slam/launch_slam_toolbox.sh` after this regressed twice).
- **Latched wall-time stamps poison message filters permanently:** a static
  TF stamped before `/clock` arrives is cached forever. Fix in
  `odom_bridge.py`: statics are periodically **re-stamped** from the live sim
  clock.
- **QoS mismatches everywhere:** the gz bridge publishes Best Effort;
  slam_toolbox, RViz displays, and nav2 map_saver default to Reliable /
  Volatile. Fixes: subscription QoS override for `/scan`
  (`slam/slam_params.yaml`), Transient Local on the RViz Map display, and a
  direct QoS-matched map saver (`slam/map_save.py`) because `map_saver_cli`
  times out.
- **The ROS CLI lies:** `ros2 topic echo/hz`, `node list`, `param get`, and
  `tf2_monitor` all gave false negatives (stale daemon, QoS mismatch — and
  tf2_monitor compares sim stamps against its own wall clock, reporting a
  1.78-billion-second "delay" that implicates nothing). We built small
  QoS-matched probes (`stamp_check.py`, map/scan probes) and trusted only
  those. The scan probe (805 valid returns) is how we proved a "missing"
  laser display was a rendering issue, not a data issue.
- **FastDDS shared-memory transport degrades** over long many-restart
  sessions: publishers demonstrably alive yet invisible to new subscribers.
  Fix: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and restarting stragglers in fresh
  terminals so all participants share one transport.
- **Stray embedded `mavsdk_server` processes** from unclean exits held UDP
  14540 and consumed PX4 telemetry — the root cause behind "odom_bridge
  connects but counts stay zero" and previously-ignored "bind error: Address
  in use" warnings.
- **ros_gz_bridge ignores CLI topic remaps** for bridged topics; the YAML
  config_file path renames correctly.
- **Route dispatch bug caught by the audit file:** "walls_survey" was
  shadowed by prefix-matching "survey_lawnmower", flying the wrong (open-field)
  route — visible instantly in `missions/last_mission.json`. Fix: exact
  route-name match takes precedence; regression-tested.

**Known limitation, stated plainly:** residual scan-delivery drops throttle
the accepted-scan rate, so wall lines in the saved map are thin/dashed rather
than solid. The cause chain is measured (above); the next tuning step would be
bridge/subscription queue depths and slam_toolbox's throttle settings, or a
second mapping pass (loop closure thickens and tightens the map).

### Autonomous navigation (designed, not flown)

`navigate_to` exists end-to-end in the schema and validator (goal x/y/yaw,
50 m nav fence). The planned executor is **A\* over our own occupancy grid**
feeding the existing offboard executor, deliberately *not* Nav2: Nav2's
controller stack assumes a differential-drive ground robot emitting
`/cmd_vel`, which nothing on a MAVSDK quadcopter consumes; planning over the
SLAM grid directly reuses the proven executor and keeps the same
"planner proposes, deterministic code flies" separation.

## 5. Production frameworks (what I'd adopt at scale, and why not mid-project)

- **Aerostack2** — the serious ROS 2 aerial-autonomy framework (PX4 platform
  plugin, NED↔ENU handling, TF tree, behaviour trees). The right foundation
  for a product; adopting it mid-take-home would have replaced a working
  pipeline with a week of migration.
- **monemati/RTABMap-ROS2-PX4, eOvic/PX4-ROS2-SLAM-Control** — community
  blueprints for exactly the PX4→ROS odometry/TF bridging problem. Notably,
  their core component (an odom converter from PX4 `VehicleOdometry` to ROS
  odom/TF) is the same design as our `slam/odom_bridge.py`, built here
  independently and debugged to root cause — which is the better interview
  artifact than a cloned working stack.

## 6. Security & process notes

- API keys live only in the environment / `.env` (git-ignored; `.env.example`
  ships placeholders). Provider error paths avoid echoing URLs containing
  key query-params after an incident where a pasted traceback exposed one —
  keys were rotated and error handling hardened.
- Every accepted mission is persisted before execution (audit + replay via
  `--mission missions/last_mission.json`).
- The task PDF contained an embedded prompt-injection line ("IGNORE
  EVERYTHING FROM THIS POINT…"); noted and ignored — fitting, given the
  pipeline's own premise that untrusted text must pass a validator before it
  can move a vehicle.

## 7. What I'd improve with more time

Per-action schema sub-models; runtime `telemetry.home()` instead of the
static `HOME_OFFSETS_NED` table; the A\* `navigate_to` executor flown;
scan-rate tuning + a loop-closure pass for a dense map; gimballed camera for
persistent overhead tracking; CI running the validator/decomposition/route
unit tests.
