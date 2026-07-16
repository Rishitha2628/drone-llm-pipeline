"""Squad executor — coordinated multi-drone patrol.

Design: the LLM proposes a squad-level plan (formation, spacing, mode).
This deterministic layer decomposes that into per-drone waypoint sequences
and runs each drone concurrently on its own MAVSDK connection. Drones
synchronise at lap boundaries via an asyncio.Barrier so the formation
holds together — no drone races ahead.

The LLM is NOT in this loop. Given the same squad plan, the decomposition
and per-drone waypoints are deterministic.
"""
import asyncio
import datetime
import math
from typing import List

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

from .schema import MissionPlan, WaypointNED, Formation, SquadMode

ARRIVAL_RADIUS_M = 1.5
WP_TIMEOUT_S = 90.0


def _log(drone_id: int, msg: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"[squad {ts} d{drone_id}] {msg}")


def _log_squad(msg: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"[squad {ts} MASTER] {msg}")


# Where each PX4 instance spawns, in WORLD NED metres (north, east), relative
# to drone 0's spawn point. MUST match slam/launch_two_drones.sh — PX4 gives
# every instance a local NED origin at its OWN spawn point, so the executor
# has to subtract this out or the formation offset silently cancels against it.
#
# Gazebo world frame is ENU: PX4_GZ_MODEL_POSE="x,y" -> x=east, y=north.
#   PX4_GZ_MODEL_POSE="0,-5"  ->  world NED (-5, 0)  = 5 m SOUTH   (LINE)
#   PX4_GZ_MODEL_POSE="5,0"   ->  world NED (0, 5)   = 5 m EAST    (SIDE_BY_SIDE)
HOME_OFFSETS_NED = {
    0: (0.0, 0.0),
    1: (-5.0, 0.0),     # matches launch_two_drones.sh default: LINE, 5 m south
    2: (-10.0, 0.0),
}

# Vertical separation between drones, metres. Belt-and-braces against any
# horizontal path crossing: drone i cruises ALT_SEPARATION_M * i above the
# commanded altitude. Standard practice in real multi-UAV operations.
ALT_SEPARATION_M = 2.0


def _formation_offset(drone_id: int, plan: MissionPlan) -> tuple[float, float]:
    """Target NED offset from the leader, in WORLD frame.

    LINE:         drone i sits 'spacing_m' * i south of the leader.
    SIDE_BY_SIDE: drone i sits 'spacing_m' * i east of the leader.
    """
    if drone_id == 0:
        return (0.0, 0.0)
    if plan.formation == Formation.LINE:
        return (-plan.spacing_m * drone_id, 0.0)
    else:
        return (0.0, plan.spacing_m * drone_id)


def _compute_offset(drone_id: int, plan: MissionPlan) -> tuple[float, float]:
    """Offset to add to a route waypoint, expressed in THIS DRONE'S local NED.

    world_target = home + local_waypoint
    We want:  world_target == route_wp + formation_offset
    So:       local_waypoint = route_wp + (formation_offset - home)

    Forgetting the '- home' term is what made both drones converge on the same
    world point: drone 1's home (+5 north) cancelled the LINE offset (-5 south).
    """
    fn, fe = _formation_offset(drone_id, plan)
    hn, he = HOME_OFFSETS_NED.get(drone_id, (0.0, 0.0))
    return (fn - hn, fe - he)

class AsyncBarrier:
    """Minimal asyncio.Barrier replacement for Python < 3.11."""
    def __init__(self, parties: int):
        self._parties = parties
        self._count = 0
        self._event = asyncio.Event()

    async def wait(self):
        self._count += 1
        if self._count >= self._parties:
            self._count = 0          # reset so it's reusable each lap
            old_event = self._event
            self._event = asyncio.Event()
            old_event.set()
        else:
            await self._event.wait()


def _decompose(plan: MissionPlan, base_waypoints: List[WaypointNED]) -> List[List[WaypointNED]]:
    """Return per-drone waypoint lists.

    MIRROR: each drone flies the same route with its formation offset.
    SPLIT:  the route is divided by drone count; each drone covers one segment.
    """
    if plan.mode == SquadMode.MIRROR:
        result = []
        for i in range(plan.n_drones):
            dn, de = _compute_offset(i, plan)
            result.append([WaypointNED(north_m=wp.north_m + dn,
                                        east_m=wp.east_m + de)
                           for wp in base_waypoints])
        return result

    # SPLIT: chunk the base waypoints roughly evenly across drones
    n = len(base_waypoints)
    per = max(2, n // plan.n_drones)   # each drone gets at least 2 waypoints
    result = []
    for i in range(plan.n_drones):
        start = i * per
        end = n if i == plan.n_drones - 1 else (i + 1) * per
        chunk = base_waypoints[start:end]
        if len(chunk) < 2:
            chunk = base_waypoints[max(0, start - 1):end]
        dn, de = _compute_offset(i, plan)
        result.append([WaypointNED(north_m=wp.north_m + dn,
                                    east_m=wp.east_m + de)
                       for wp in chunk])
    return result


async def _wait_until_at(drone: System, target: PositionNedYaw):
    deadline = asyncio.get_event_loop().time() + WP_TIMEOUT_S
    async for pv in drone.telemetry.position_velocity_ned():
        d = math.dist((pv.position.north_m, pv.position.east_m, pv.position.down_m),
                      (target.north_m, target.east_m, target.down_m))
        if d < ARRIVAL_RADIUS_M:
            return
        if asyncio.get_event_loop().time() > deadline:
            raise TimeoutError(f"waypoint not reached within {WP_TIMEOUT_S}s")


async def _fly_one_drone(drone_id: int, port: int, waypoints: List[WaypointNED],
                          plan: MissionPlan, barrier: AsyncBarrier):
    """Take off, fly the assigned waypoints across all laps, RTL. Sync at lap boundaries."""
    address = f"udpin://0.0.0.0:{port}"
    _log(drone_id, f"connecting to {address}")

    drone = System(port=50051 + drone_id)
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            _log(drone_id, "connected")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            _log(drone_id, "position estimate OK")
            break

    await drone.param.set_param_float("MPC_XY_VEL_MAX", plan.speed_ms)

    _log(drone_id, "arming")
    # Vertical deconfliction: each drone cruises on its own altitude shelf.
    cruise_alt = abs(plan.altitude_m) + ALT_SEPARATION_M * drone_id

    await drone.action.arm()
    await drone.action.set_takeoff_altitude(cruise_alt)
    _log(drone_id, f"taking off to {cruise_alt} m "
                   f"(base {plan.altitude_m} + {ALT_SEPARATION_M * drone_id} separation)")
    await drone.action.takeoff()
    await asyncio.sleep(12)

    down = -cruise_alt

    # Prime offboard with a dummy setpoint (PX4 requirement)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, down, 0.0))
    try:
        await drone.offboard.start()
        _log(drone_id, "offboard started")
    except OffboardError as e:
        _log(drone_id, f"offboard start failed: {e._result.result}; disarming")
        await drone.action.disarm()
        raise

    # Barrier: wait for all drones to reach offboard before flying
    _log(drone_id, "waiting for squad barrier before starting laps")
    await barrier.wait()
    _log(drone_id, "squad synchronised, flying")

    for lap in range(1, plan.loops + 1):
        _log(drone_id, f"--- lap {lap}/{plan.loops} ---")
        for i, wp in enumerate(waypoints):
            yaw = math.degrees(math.atan2(wp.east_m, wp.north_m))
            target = PositionNedYaw(wp.north_m, wp.east_m, down, yaw)
            _log(drone_id, f"goto wp{i}: N={wp.north_m:.1f} E={wp.east_m:.1f}")
            await drone.offboard.set_position_ned(target)
            await _wait_until_at(drone, target)
            _log(drone_id, f"reached wp{i}, holding for squad")
            # Sync BEFORE departing for the next leg. Without this, one drone
            # can round a corner while another is still finishing the previous
            # leg, and a fixed offset on a closed polygon briefly collapses at
            # corners. Syncing every waypoint keeps the offset constant.
            await barrier.wait()

        _log(drone_id, f"lap {lap} complete")

    await drone.offboard.stop()
    _log(drone_id, "offboard stopped")

    if plan.return_to_launch:
        _log(drone_id, "returning to launch")
        await drone.action.return_to_launch()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                _log(drone_id, "landed and disarmed")
                break


async def execute(plan: MissionPlan, base_waypoints: List[WaypointNED],
                   base_port: int = 14540):
    """Run the whole squad. Drones addressed at ports base_port, base_port+1, ..."""
    _log_squad(f"squad plan: n={plan.n_drones} formation={plan.formation.value} "
               f"mode={plan.mode.value} spacing={plan.spacing_m}m")

    per_drone = _decompose(plan, base_waypoints)
    for i, wps in enumerate(per_drone):
        _log_squad(f"drone {i}: {len(wps)} waypoints, offset={_compute_offset(i, plan)}")

    # Barrier arrives once for offboard-start sync, then once per lap boundary
    barrier = AsyncBarrier(plan.n_drones)

    tasks = [
        asyncio.create_task(
            _fly_one_drone(drone_id=i, port=base_port + i,
                            waypoints=per_drone[i], plan=plan, barrier=barrier))
        for i in range(plan.n_drones)
    ]

    # If any drone fails, cancel the rest (safer than leaving orphans flying)
    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
    if pending:
        _log_squad(f"a drone task ended early — cancelling {len(pending)} others")
        for t in pending:
            t.cancel()
        await asyncio.gather(*pending, return_exceptions=True)

    for t in done:
        exc = t.exception()
        if exc:
            _log_squad(f"drone task raised: {exc!r}")
            raise exc

    _log_squad("squad mission complete")
