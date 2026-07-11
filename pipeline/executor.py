"""Deterministic executor: validated MissionPlan -> MAVSDK offboard commands.

Properties this layer guarantees:
  * Deterministic: the same mission JSON always produces the same command
    sequence (no LLM, no randomness, no hidden state).
  * Auditable: every command issued is logged with a timestamp.
  * Bounded: it can only fly waypoints that already passed the validator.
"""
import asyncio
import datetime
import math
from typing import List

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

from .schema import MissionPlan, WaypointNED

ARRIVAL_RADIUS_M = 1.5
WP_TIMEOUT_S = 60.0


def _log(msg: str):
    print(f"[executor {datetime.datetime.now().strftime('%H:%M:%S')}] {msg}")


async def _wait_until_at(drone: System, target: PositionNedYaw):
    deadline = asyncio.get_event_loop().time() + WP_TIMEOUT_S
    async for pv in drone.telemetry.position_velocity_ned():
        pos = pv.position
        d = math.dist((pos.north_m, pos.east_m, pos.down_m),
                      (target.north_m, target.east_m, target.down_m))
        if d < ARRIVAL_RADIUS_M:
            return
        if asyncio.get_event_loop().time() > deadline:
            raise TimeoutError(f"waypoint not reached within {WP_TIMEOUT_S}s")


async def execute(plan: MissionPlan, waypoints: List[WaypointNED],
                  system_address: str = "udp://:14540"):
    down = -abs(plan.altitude_m)  # NED: down is negative altitude

    drone = System()
    _log(f"connecting to {system_address}")
    await drone.connect(system_address=system_address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            _log("vehicle connected")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            _log("position estimate OK")
            break

    await drone.param.set_param_float("MPC_XY_VEL_MAX", plan.speed_ms)
    _log(f"max speed set to {plan.speed_ms} m/s")

    _log("arming")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(abs(plan.altitude_m))
    _log(f"taking off to {plan.altitude_m} m")
    await drone.action.takeoff()
    await asyncio.sleep(12)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, down, 0.0))
    try:
        await drone.offboard.start()
        _log("offboard started")
    except OffboardError as e:
        _log(f"offboard start failed: {e._result.result}; disarming")
        await drone.action.disarm()
        raise

    for lap in range(1, plan.loops + 1):
        _log(f"--- lap {lap}/{plan.loops} ---")
        for i, wp in enumerate(waypoints):
            yaw = math.degrees(math.atan2(wp.east_m, wp.north_m))
            target = PositionNedYaw(wp.north_m, wp.east_m, down, yaw)
            _log(f"goto wp{i}: N={wp.north_m} E={wp.east_m} D={down}")
            await drone.offboard.set_position_ned(target)
            await _wait_until_at(drone, target)
            _log(f"reached wp{i}")

    await drone.offboard.stop()
    _log("offboard stopped")

    if plan.return_to_launch:
        _log("returning to launch")
        await drone.action.return_to_launch()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                _log("landed and disarmed — mission complete")
                break
