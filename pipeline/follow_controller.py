"""Follow controller — deterministic P-controller for target following.

Reads /tmp/target_state.json (published by vision.detector) at ~10 Hz.
Converts bounding-box error into MAVSDK body-frame velocity setpoints:
  - Yaw rate from horizontal centroid error (target left/right of centre)
  - Forward speed from bbox area (small bbox = far = go forward; large = close = stop)
  - Altitude hold via vertical velocity toward target altitude
Loss of target for >LOSS_HOVER_S seconds -> hover in place.
Loss of target for >LOSS_TIMEOUT_S seconds -> RTL.
Hard cap: FOLLOW_DURATION_S seconds total, then RTL.

The LLM is NOT in this loop. The only inputs are (a) the mission plan
(follow duration, target class, altitude), (b) the detector's bbox state,
(c) live telemetry. Everything is deterministic given those inputs.
"""
import asyncio
import datetime
import math
import time

from mavsdk import System
from mavsdk.offboard import (
    OffboardError, VelocityBodyYawspeed, PositionNedYaw,
)

from .schema import MissionPlan
from .target_state import read_state, clear_state

# ---- Tunable constants -------------------------------------------------
CONTROL_HZ         = 10.0                     # setpoint update rate
LOSS_HOVER_S       = 1.5                      # seconds w/o target before hover
LOSS_TIMEOUT_S     = 8.0                      # seconds w/o target before RTL
DESIRED_AREA_FRAC  = 0.06                     # keep bbox at ~6% of frame
MAX_FORWARD_MS     = 3.0                      # cap forward speed while following
MAX_BACK_MS        = 1.5                      # cap backward speed if too close
KP_YAW_DEG_S       = 45.0                     # yaw rate at max horizontal error
KP_FORWARD_MS      = 4.0                      # forward gain per (desired - actual) area frac
DEADBAND_ERR       = 0.05                     # ignore tiny centring errors
# ------------------------------------------------------------------------


def _log(msg: str):
    print(f"[follow {datetime.datetime.now().strftime('%H:%M:%S')}] {msg}")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


async def _await_position(drone: System):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            return


async def follow(plan: MissionPlan, system_address: str = "udpin://0.0.0.0:14540"):
    """Execute a follow_target mission."""
    down = -abs(plan.altitude_m)
    target_cls = (plan.target_class or "").lower()
    follow_duration = plan.follow_duration_s
    search_yaw = plan.search_yaw_rate_deg_s

    clear_state()  # forget any stale bbox from a previous run

    drone = System()
    _log(f"connecting to {system_address}")
    await drone.connect(system_address=system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            _log("vehicle connected")
            break

    await _await_position(drone)
    _log("position estimate OK")

    _log("arming")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(abs(plan.altitude_m))
    _log(f"taking off to {plan.altitude_m} m")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # Prime offboard with a zero velocity setpoint (required by PX4)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        _log("offboard started (velocity-body mode)")
    except OffboardError as e:
        _log(f"offboard start failed: {e._result.result}; disarming")
        await drone.action.disarm()
        raise

    # -------- follow loop --------
    dt = 1.0 / CONTROL_HZ
    t_start = time.time()
    t_last_seen = 0.0
    seen_ever = False
    state_last = "SEARCH"

    while True:
        elapsed = time.time() - t_start
        if elapsed > follow_duration:
            _log(f"follow duration ({follow_duration:.0f}s) elapsed, ending")
            break

        target = read_state()
        if target and target.cls == target_cls:
            t_last_seen = time.time()
            seen_ever = True

            # --- yaw from horizontal centring error ---
            err = target.horizontal_error_norm()      # -1 .. +1
            if abs(err) < DEADBAND_ERR:
                yaw_rate = 0.0
            else:
                yaw_rate = _clamp(err * KP_YAW_DEG_S, -KP_YAW_DEG_S, KP_YAW_DEG_S)

            # --- forward from bbox size (distance proxy) ---
            area_err = DESIRED_AREA_FRAC - target.area_fraction  # +ve => too far, go forward
            fwd = _clamp(area_err * KP_FORWARD_MS * 20.0, -MAX_BACK_MS, MAX_FORWARD_MS)

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_m_s=fwd, right_m_s=0.0,
                                     down_m_s=0.0, yawspeed_deg_s=yaw_rate))

            if state_last != "FOLLOW":
                _log(f"LOCKED target '{target.cls}' conf={target.conf:.2f}")
                state_last = "FOLLOW"

        else:
            time_lost = time.time() - t_last_seen if t_last_seen else float("inf")

            if time_lost > LOSS_TIMEOUT_S:
                _log(f"target lost for {time_lost:.1f}s > {LOSS_TIMEOUT_S}s, ending follow")
                break

            if not seen_ever or time_lost > LOSS_HOVER_S:
                # Searching: hover in place, yaw slowly to scan
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, search_yaw))
                if state_last != "SEARCH":
                    _log(f"SEARCHING (yaw {search_yaw:.0f} deg/s)")
                    state_last = "SEARCH"
            else:
                # Brief flicker: hover, no yaw
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                if state_last != "HOVER":
                    _log(f"target flicker, hover ({time_lost:.1f}s)")
                    state_last = "HOVER"

        await asyncio.sleep(dt)

    # -------- teardown --------
    _log("stopping offboard")
    await drone.offboard.stop()

    if plan.return_to_launch:
        _log("returning to launch")
        await drone.action.return_to_launch()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                _log("landed and disarmed — follow mission complete")
                break
