# Drone LLM Pipeline

Natural-language drone (and multi-drone) command system:
**prompt → LLM planner → validated JSON → deterministic executor → PX4 SITL / Gazebo Harmonic.**

All three senior challenges are implemented and demonstrated:

| Deliverable | Status | Evidence |
|---|---|---|
| Core task — NL patrol missions | ✅ working | `demo/` video, mission audit logs |
| Challenge 1 — multi-drone formations | ✅ working | 2-drone line & side-by-side, mirror/split modes |
| Challenge 2 — SLAM (online mapping + localization) | ✅ working | slam_toolbox occupancy map, `docs/evidence/walls_map.pgm` |
| Challenge 3 — vision detect + follow | ✅ working | YOLOv8 detection JPGs, moving-target follow video |
| LLM providers | ✅ Gemini live-tested (gemini-3.1-flash-lite); Groq, Anthropic wired | env-switchable |

See `docs/APPROACH.md` for architecture, design decisions, and an honest account of
what was hard (especially Challenge 2).

---

## 1. Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 | tested natively |
| PX4-Autopilot | v1.15+ (Gazebo Harmonic / `gz sim`) | built at `~/PX4-Autopilot` |
| ROS 2 | Humble | needed for Challenge 2 (SLAM) only |
| Python | 3.10 | system python for ROS interop |
| Docker + Compose v2 | any recent | optional — runs the pipeline container |
| NVIDIA GPU | optional | hybrid-graphics laptops need the PRIME env vars below |

```bash
# Python deps (native path) — versions pinned to the tested environment
pip3 install --user mavsdk==3.15.3 pydantic==2.13.4 pyyaml==6.0.2 \
                    requests==2.32.3 ultralytics==8.4.92 \
                    opencv-python==4.10.0.84 numpy==1.26.4

# ROS 2 packages for Challenge 2
sudo apt install ros-humble-slam-toolbox ros-humble-ros-gz-bridge \
                 ros-humble-tf2-ros ros-humble-nav2-map-server \
                 ros-humble-diagnostic-updater
```

### Hybrid graphics (AMD iGPU + NVIDIA dGPU)

Every Gazebo *and* RViz launch on such machines needs:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb <command>
```

Without it, Gazebo may render on the wrong GPU (blank window) and RViz point
displays silently fail to draw.

### DDS transport (important for long sessions)

FastDDS shared-memory transport degrades across many node restarts. Force UDP once:

```bash
echo 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4' >> ~/.bashrc
```

---

## 2. LLM configuration

Copy the template and add a key for **one** provider (Gemini free tier is enough):

```bash
cp .env.example .env    # then edit
# or export directly:
export LLM_PROVIDER=gemini
export GEMINI_API_KEY="your-key"           # aistudio.google.com
export PLANNER_MODEL="gemini-3.1-flash-lite"   # current free-tier model
```

> **Model availability:** Google gates older models for newly-created keys — a
> 404 naming the model ("no longer available to new users") means exactly that.
> List what your key can call:
> `curl -s "https://generativelanguage.googleapis.com/v1beta/models?key=$GEMINI_API_KEY"`
> and pick a current flash-tier model.

Every mission also runs fully offline with `--no-llm` (deterministic keyword
parser, same JSON contract) — the examiner needs **no API key** to fly anything.

---

## 3. Quick start — core task

Terminal A (sim — keep open; closing it kills PX4):

```bash
cd ~/PX4-Autopilot
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb \
  make px4_sitl gz_x500
```

Terminal B — fly:

```bash
cd drone-llm-pipeline
# LLM-planned mission (the headline path):
python3 -m pipeline.main "check the fence line, keep it around 12 meters, come back when done"
# no API key? identical JSON contract via the offline parser:
python3 -m pipeline.main --no-llm "Patrol the perimeter loop twice at 15 metres"
# validation only (no sim needed):
python3 -m pipeline.main --dry-run "..."
```

Every validated mission is audited to `missions/last_mission.json`.

### Fully containerized quick start (zero native installs)

```bash
docker compose --profile sim up -d px4-sim     # headless PX4 SITL (wait ~30 s for "Ready for takeoff!")
docker compose run --rm pipeline --no-llm "Patrol the perimeter loop twice at 15 metres"
docker compose --profile sim down
```

Tested end-to-end: arm → takeoff → 2 patrol laps (10 waypoints) → RTL → land,
verified in executor and PX4 logs. The sim container runs PX4 v1.14 headless
with ulogs mounted on tmpfs — a prior session filled ~60 GB of disk through
container-layer logging; the tmpfs mount makes recurrence impossible.

The pipeline container also runs against the **native** sim (same
`network_mode: host`, PX4 on `localhost:14540`), and forwards
`GEMINI_API_KEY` / `GROQ_API_KEY` / `ANTHROPIC_API_KEY` plus `LLM_PROVIDER` /
`PLANNER_MODEL` from your environment — the live-LLM path was verified running
inside the container. Do **not** run the containerized sim and a native PX4
simultaneously; they fight over port 14540.

**Containerization scope** (claims match what was tested):

| Component | Containerized? | Why |
|---|---|---|
| Core task (headless sim + pipeline) | ✅ tested end-to-end | two-command flow above |
| LLM planning (all providers) | ✅ tested in-container | env-forwarded keys |
| Challenge 1 multi-drone | ❌ native sim | second PX4 instance + port isolation not containerized |
| Challenge 3 vision | ❌ native | Gazebo camera rides shared-memory transport; not visible across containers |
| Challenge 2 SLAM | ❌ native | host ROS 2 graph (bridge / TF / slam_toolbox) |

Containerizing the remaining components (per-vehicle sim services, ROS graph
in a compose network) is follow-on packaging work; the challenge demos use the
native setup documented per-section below.

---

## 4. Challenge 3 — vision detect + follow

```bash
# Terminal A: camera drone
cd ~/PX4-Autopilot
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb \
  PX4_GZ_WORLD=lawn make px4_sitl gz_x500_mono_cam

# Terminal B: spawn a walking person (Actor) — see docs/APPROACH.md for the SDF
# (world name must match: /world/lawn/create)

# Terminal C: detector (native python, NOT venv/docker)
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 -m vision.detector --target person

# Terminal D: follow mission
python3 -m pipeline.main "take off to 8 metres and follow the person for 40 seconds"
# offline fallback: add --no-llm
```

Annotated JPGs (bounding box + TARGET DETECTED banner) land in `detections/` —
this is the "send a picture to the operator" requirement. Target class is
LLM-configurable (`target_class` field) and whitelist-enforced by the validator
(try `"follow the giraffe"` to see a rejection).

---

## 5. Challenge 1 — multi-drone formations

```bash
# Terminal A: drone 0 + Gazebo
cd ~/PX4-Autopilot
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb \
  PX4_GZ_WORLD=lawn make px4_sitl gz_x500

# Terminal B: drone 1 (attaches to the running Gazebo, spawns 5 m south)
bash slam/launch_two_drones.sh

# Clear PX4's camera-follow so both drones stay in frame:
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: ""'

# Terminal C: fly
python3 -m pipeline.main \
  "patrol the perimeter twice at 15 metres with 2 drones in a line 5m apart"
# or: "... side by side ..." / "... split the route ..."
# offline fallback: add --no-llm (same JSON contract)
```

The LLM (or offline parser) chooses formation (`line`/`side_by_side`) and mode
(`mirror`/`split`). Safety: spacing is validator-clamped to ≥2 m, drones sync at
**every waypoint** via an asyncio barrier, fly on separated altitude shelves
(+2 m per drone), and offsets are **home-aware** (each PX4 instance's NED origin
is its own spawn point — see APPROACH.md for the collision bug this prevents).

> `HOME_OFFSETS_NED` in `pipeline/squad_executor.py` must match the spawn pose
> in `slam/launch_two_drones.sh`. Production version would read
> `telemetry.home()` at runtime.

---

## 6. Challenge 2 — SLAM (2D lidar + slam_toolbox, walls world)

Startup order matters (any sim-time node started before `/clock` exists will
wall-clock-stamp and poison message filters):

```bash
# Terminal A: lidar drone in the walls world
cd ~/PX4-Autopilot
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb \
  PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d

# Terminal B: bridge (scan + clock; config renames the long gz topic to /scan)
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=slam/bridge_scan.yaml

# Terminal C: PX4 odometry -> ROS odom + TF (MUST pass sim time on the CLI)
python3 -m slam.odom_bridge --ros-args -p use_sim_time:=true
# wait for "MAVSDK connected"

# Terminal D: verification gate — all three columns live, gap < 0.1 s
python3 -m slam.stamp_check

# Terminal E: slam_toolbox (script exists because the CLI sim-time flag is
# mandatory — the params file alone does not apply it on this Humble build)
bash slam/launch_slam_toolbox.sh

# Terminal F: coverage flight past all four walls (collision-verified route)
python3 -m pipeline.main "patrol the walls_survey once at 7 metres at 2 m/s"
# offline fallback: add --no-llm

# After landing — save the map (nav2 map_saver fails on QoS; direct saver works):
python3 slam/map_save.py     # writes challenge2-evidence/walls_map.pgm/.yaml
```

RViz (launch with the NVIDIA env prefix): Fixed Frame `map`; Map display with
**Durability = Transient Local**; LaserScan `/scan` with **Reliability = Best
Effort**, Size 0.15, Decay 1; TF display shows the drone.

The saved artifact from our run: `docs/evidence/walls_map.pgm` (598 occupied
cells, 0.1 m resolution). Challenge 2 was by far the hardest part of this task —
`docs/APPROACH.md` §Challenge 2 documents every failure mode we hit and how each
was diagnosed, including why depth-camera ICP odometry is structurally
degenerate on this airframe.

---

## 7. Repository structure

```
pipeline/
  schema.py            # MissionPlan — the strict LLM<->executor contract
  validator.py         # safety limits: altitude/speed/geofence/whitelist/spacing
  planner.py           # LLM providers (gemini/groq/anthropic) + offline parser
  executor.py          # single-drone MAVSDK offboard executor
  squad_executor.py    # multi-drone decomposition + barrier-synced execution
  follow_controller.py # vision follow P-controller (yaw + forward, hover on loss)
  target_state.py      # detector -> controller shared-state contract
  routes.yaml          # named route whitelist (incl. walls_survey coverage route)
  main.py              # entry point + dispatch
vision/
  detector.py          # YOLOv8n on the Gazebo camera; saves annotated JPGs
slam/
  odom_bridge.py       # PX4 telemetry -> /odom + TF (sim-time safe, restamped statics)
  bridge_scan.yaml     # ros_gz_bridge config (scan + clock)
  slam_params.yaml     # slam_toolbox params incl. /scan QoS override
  launch_slam_toolbox.sh
  launch_two_drones.sh # second PX4 instance for Challenge 1
  stamp_check.py       # QoS-correct stamp diagnostics (the CLI tools lie)
  map_save.py          # direct /map -> PGM+YAML saver
docs/
  APPROACH.md          # architecture, decisions, per-challenge write-up, lessons
  evidence/            # map artifact, detection JPGs, screenshots
demo/                  # recorded demo videos
Dockerfile, docker-compose.yml, .env.example
```

## 8. Known gotchas (each cost us real debugging time)

- **Stray `mavsdk_server` processes** survive unclean Python exits, hold UDP
  14540, and silently eat PX4 heartbeats. Every unexplained "bind error: Address
  in use" or dead telemetry: `pkill -f mavsdk_server`.
- **Closing the PX4 terminal kills PX4** — keep Terminal A sacred.
- **`use_sim_time` must be passed on the node's command line** (`--ros-args -p
  use_sim_time:=true`); params-file-only or post-hoc setting silently fails for
  some Humble nodes.
- **`ros2 topic echo/hz` and `tf2_monitor` are unreliable** under QoS mismatch
  and stale daemons (`ros2 daemon stop`). Trust the QoS-matched probes in
  `slam/`.
- **ros_gz_bridge CLI remaps don't apply to bridged topics** — use a YAML
  `config_file` to rename.
- PX4 auto-sets **Gazebo camera-follow** on boot — clear it for formation shots.
- World name changes topic prefixes (`/world/lawn/...` vs `/world/walls/...`);
  spawn service calls against the wrong world time out silently.
