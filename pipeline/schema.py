"""Mission schema — the strict contract between LLM planner and executors."""
from enum import Enum
from typing import List, Optional
from pydantic import BaseModel, Field, model_validator


class Action(str, Enum):
    """The only commands the executors know. Anything else is rejected."""
    PATROL_ROUTE  = "patrol_route"    # single drone: fly a named route
    FLY_WAYPOINTS = "fly_waypoints"   # single drone: explicit NED waypoints
    RETURN_HOME   = "return_home"     # single drone
    FOLLOW_TARGET = "follow_target"   # single drone: vision follow
    NAVIGATE_TO   = "navigate_to"     # ground robot: Nav2 goal while SLAM maps
    SQUAD_PATROL  = "squad_patrol"    # multi-drone coordinated patrol


class WaypointNED(BaseModel):
    north_m: float = Field(..., ge=-500, le=500)
    east_m: float = Field(..., ge=-500, le=500)


class Formation(str, Enum):
    LINE = "line"              # drone i behind drone i-1
    SIDE_BY_SIDE = "side_by_side"  # drones abreast, offset in the east axis


class SquadMode(str, Enum):
    MIRROR = "mirror"    # all drones fly the same route with a NED offset
    SPLIT  = "split"     # route is split between drones; each covers a portion


ALLOWED_TARGET_CLASSES = {
    "person", "car", "truck", "bus", "motorcycle", "bicycle",
    "dog", "cat", "sheep", "cow", "horse",
}


class MissionPlan(BaseModel):
    """A complete, self-contained mission. Same JSON in -> same behaviour out."""
    action: Action
    route: Optional[str] = Field(None)
    waypoints: List[WaypointNED] = Field(default_factory=list, max_length=50)
    altitude_m: float = Field(10.0)
    speed_ms: float = Field(5.0)
    loops: int = Field(1)
    return_to_launch: bool = Field(True)

    # follow_target
    target_class: Optional[str] = Field(None)
    follow_duration_s: float = Field(30.0)
    search_yaw_rate_deg_s: float = Field(20.0)

    # navigate_to
    goal_x_m: Optional[float] = Field(None, ge=-200, le=200)
    goal_y_m: Optional[float] = Field(None, ge=-200, le=200)
    goal_yaw_deg: float = Field(0.0, ge=-180, le=180)

    # squad_patrol
    n_drones: int = Field(2, ge=2, le=3, description="Number of drones in the squad")
    formation: Formation = Field(Formation.LINE)
    spacing_m: float = Field(5.0, description="Separation between adjacent drones in metres")
    mode: SquadMode = Field(SquadMode.MIRROR)

    reasoning: str = Field("", max_length=500)

    @model_validator(mode="after")
    def action_requirements(self):
        if self.action == Action.PATROL_ROUTE and not self.route:
            raise ValueError("patrol_route requires a 'route' name")
        if self.action == Action.FLY_WAYPOINTS and len(self.waypoints) < 2:
            raise ValueError("fly_waypoints requires at least 2 waypoints")
        if self.action == Action.FOLLOW_TARGET and not self.target_class:
            raise ValueError("follow_target requires a 'target_class'")
        if self.action == Action.NAVIGATE_TO and (self.goal_x_m is None or self.goal_y_m is None):
            raise ValueError("navigate_to requires 'goal_x_m' and 'goal_y_m'")
        if self.action == Action.SQUAD_PATROL and not self.route:
            raise ValueError("squad_patrol requires a 'route' name")
        return self


def schema_for_prompt() -> str:
    import json
    return json.dumps(MissionPlan.model_json_schema(), indent=2)
