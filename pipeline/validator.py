"""Guardrail layer: schema + safety/sanity validation."""
from dataclasses import dataclass
from typing import List, Tuple

import yaml

from .schema import MissionPlan, Action, WaypointNED, ALLOWED_TARGET_CLASSES


@dataclass(frozen=True)
class SafetyLimits:
    min_altitude_m: float = 2.0
    max_altitude_m: float = 50.0
    min_speed_ms: float = 0.5
    max_speed_ms: float = 12.0
    max_loops: int = 10
    max_waypoints: int = 50
    geofence_radius_m: float = 200.0
    # follow_target specifics
    max_follow_duration_s: float = 120.0
    max_search_yaw_rate_deg_s: float = 45.0


LIMITS = SafetyLimits()


class ValidationError(Exception):
    def __init__(self, errors: List[str]):
        self.errors = errors
        super().__init__("; ".join(errors))


def load_routes(path: str = "pipeline/routes.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)["routes"]


def resolve_waypoints(plan: MissionPlan, routes: dict) -> List[WaypointNED]:
    if plan.action == Action.PATROL_ROUTE:
        pts = routes[plan.route]["waypoints"]
        return [WaypointNED(north_m=p[0], east_m=p[1]) for p in pts]
    return plan.waypoints


def validate(plan: MissionPlan, routes: dict) -> Tuple[MissionPlan, List[WaypointNED]]:
    errors: List[str] = []

    if not (LIMITS.min_altitude_m <= plan.altitude_m <= LIMITS.max_altitude_m):
        errors.append(
            f"altitude_m={plan.altitude_m} outside allowed range "
            f"[{LIMITS.min_altitude_m}, {LIMITS.max_altitude_m}]")

    if not (LIMITS.min_speed_ms <= plan.speed_ms <= LIMITS.max_speed_ms):
        errors.append(
            f"speed_ms={plan.speed_ms} outside allowed range "
            f"[{LIMITS.min_speed_ms}, {LIMITS.max_speed_ms}]")

    if not (1 <= plan.loops <= LIMITS.max_loops):
        errors.append(f"loops={plan.loops} outside allowed range [1, {LIMITS.max_loops}]")

    if plan.action == Action.PATROL_ROUTE and plan.route not in routes:
        errors.append(f"unknown route '{plan.route}'; known routes: {sorted(routes)}")

    # follow_target checks
    if plan.action == Action.FOLLOW_TARGET:
        if plan.target_class and plan.target_class.lower() not in ALLOWED_TARGET_CLASSES:
            errors.append(
                f"target_class '{plan.target_class}' not in allowed whitelist "
                f"{sorted(ALLOWED_TARGET_CLASSES)}")
        if not (1.0 <= plan.follow_duration_s <= LIMITS.max_follow_duration_s):
            errors.append(
                f"follow_duration_s={plan.follow_duration_s} outside allowed range "
                f"[1.0, {LIMITS.max_follow_duration_s}]")
        if not (1.0 <= plan.search_yaw_rate_deg_s <= LIMITS.max_search_yaw_rate_deg_s):
            errors.append(
                f"search_yaw_rate_deg_s={plan.search_yaw_rate_deg_s} outside "
                f"allowed range [1.0, {LIMITS.max_search_yaw_rate_deg_s}]")

    waypoints = []
    if plan.action in (Action.PATROL_ROUTE, Action.FLY_WAYPOINTS):
        if not errors or plan.action != Action.PATROL_ROUTE:
            try:
                waypoints = resolve_waypoints(plan, routes)
            except KeyError:
                pass

    if len(waypoints) > LIMITS.max_waypoints:
        errors.append(f"{len(waypoints)} waypoints exceeds max {LIMITS.max_waypoints}")

    for i, wp in enumerate(waypoints):
        dist = (wp.north_m ** 2 + wp.east_m ** 2) ** 0.5
        if dist > LIMITS.geofence_radius_m:
            errors.append(
                f"waypoint {i} is {dist:.0f} m from home, outside the "
                f"{LIMITS.geofence_radius_m:.0f} m geofence")

    if errors:
        raise ValidationError(errors)
    return plan, waypoints
