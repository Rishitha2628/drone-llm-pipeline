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
    # follow_target
    max_follow_duration_s: float = 120.0
    max_search_yaw_rate_deg_s: float = 45.0
    # navigate_to
    nav_goal_radius_m: float = 50.0
    # squad_patrol
    min_squad_spacing_m: float = 2.0    # anti-collision
    max_squad_spacing_m: float = 30.0
    max_n_drones: int = 3


LIMITS = SafetyLimits()


class ValidationError(Exception):
    def __init__(self, errors: List[str]):
        self.errors = errors
        super().__init__("; ".join(errors))


def load_routes(path: str = "pipeline/routes.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)["routes"]


def resolve_waypoints(plan: MissionPlan, routes: dict) -> List[WaypointNED]:
    if plan.action in (Action.PATROL_ROUTE, Action.SQUAD_PATROL):
        pts = routes[plan.route]["waypoints"]
        return [WaypointNED(north_m=p[0], east_m=p[1]) for p in pts]
    return plan.waypoints


def validate(plan: MissionPlan, routes: dict) -> Tuple[MissionPlan, List[WaypointNED]]:
    errors: List[str] = []

    if plan.action in (Action.PATROL_ROUTE, Action.FLY_WAYPOINTS, Action.RETURN_HOME,
                       Action.FOLLOW_TARGET, Action.SQUAD_PATROL):
        if not (LIMITS.min_altitude_m <= plan.altitude_m <= LIMITS.max_altitude_m):
            errors.append(f"altitude_m={plan.altitude_m} outside allowed range "
                          f"[{LIMITS.min_altitude_m}, {LIMITS.max_altitude_m}]")
        if not (LIMITS.min_speed_ms <= plan.speed_ms <= LIMITS.max_speed_ms):
            errors.append(f"speed_ms={plan.speed_ms} outside allowed range "
                          f"[{LIMITS.min_speed_ms}, {LIMITS.max_speed_ms}]")
        if not (1 <= plan.loops <= LIMITS.max_loops):
            errors.append(f"loops={plan.loops} outside allowed range [1, {LIMITS.max_loops}]")

    if plan.action in (Action.PATROL_ROUTE, Action.SQUAD_PATROL) and plan.route not in routes:
        errors.append(f"unknown route '{plan.route}'; known routes: {sorted(routes)}")

    if plan.action == Action.FOLLOW_TARGET:
        if plan.target_class and plan.target_class.lower() not in ALLOWED_TARGET_CLASSES:
            errors.append(f"target_class '{plan.target_class}' not in whitelist "
                          f"{sorted(ALLOWED_TARGET_CLASSES)}")
        if not (1.0 <= plan.follow_duration_s <= LIMITS.max_follow_duration_s):
            errors.append(f"follow_duration_s={plan.follow_duration_s} outside "
                          f"[1.0, {LIMITS.max_follow_duration_s}]")
        if not (1.0 <= plan.search_yaw_rate_deg_s <= LIMITS.max_search_yaw_rate_deg_s):
            errors.append(f"search_yaw_rate_deg_s={plan.search_yaw_rate_deg_s} "
                          f"outside [1.0, {LIMITS.max_search_yaw_rate_deg_s}]")

    if plan.action == Action.NAVIGATE_TO and plan.goal_x_m is not None and plan.goal_y_m is not None:
        dist = (plan.goal_x_m ** 2 + plan.goal_y_m ** 2) ** 0.5
        if dist > LIMITS.nav_goal_radius_m:
            errors.append(f"navigate_to goal is {dist:.1f} m from origin, "
                          f"outside the {LIMITS.nav_goal_radius_m:.0f} m nav fence")

    # squad_patrol specific
    if plan.action == Action.SQUAD_PATROL:
        if plan.n_drones > LIMITS.max_n_drones:
            errors.append(f"n_drones={plan.n_drones} exceeds max {LIMITS.max_n_drones}")
        if not (LIMITS.min_squad_spacing_m <= plan.spacing_m <= LIMITS.max_squad_spacing_m):
            errors.append(f"spacing_m={plan.spacing_m} outside collision-safe range "
                          f"[{LIMITS.min_squad_spacing_m}, {LIMITS.max_squad_spacing_m}]")

    # waypoint resolution + geofence
    waypoints = []
    if plan.action in (Action.PATROL_ROUTE, Action.FLY_WAYPOINTS, Action.SQUAD_PATROL):
        try:
            waypoints = resolve_waypoints(plan, routes)
        except KeyError:
            pass

    if len(waypoints) > LIMITS.max_waypoints:
        errors.append(f"{len(waypoints)} waypoints exceeds max {LIMITS.max_waypoints}")

    for i, wp in enumerate(waypoints):
        # For squad, the far drones are offset — check the farthest possible offset
        offset = 0.0
        if plan.action == Action.SQUAD_PATROL:
            offset = (plan.n_drones - 1) * plan.spacing_m
        dist = ((wp.north_m + offset) ** 2 + (wp.east_m + offset) ** 2) ** 0.5
        if dist > LIMITS.geofence_radius_m:
            errors.append(f"waypoint {i} (worst-case offset) at {dist:.0f} m > "
                          f"{LIMITS.geofence_radius_m:.0f} m geofence")

    if errors:
        raise ValidationError(errors)
    return plan, waypoints
