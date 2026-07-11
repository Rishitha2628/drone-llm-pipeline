"""Mission schema — the strict contract between the LLM planner and the executor.

The LLM proposes a MissionPlan as JSON. Nothing executes unless the JSON
parses against this schema AND passes the safety checks in validator.py.
"""
from enum import Enum
from typing import List, Optional
from pydantic import BaseModel, Field, model_validator


class Action(str, Enum):
    """The only commands the executor knows. Anything else is rejected."""
    PATROL_ROUTE  = "patrol_route"
    FLY_WAYPOINTS = "fly_waypoints"
    RETURN_HOME   = "return_home"
    FOLLOW_TARGET = "follow_target"


class WaypointNED(BaseModel):
    """Local NED waypoint relative to home (metres). Down is negative-up."""
    north_m: float = Field(..., ge=-500, le=500)
    east_m: float = Field(..., ge=-500, le=500)


# YOLO class names the mission is allowed to name as a target. Whitelist.
ALLOWED_TARGET_CLASSES = {
    "person", "car", "truck", "bus", "motorcycle", "bicycle",
    "dog", "cat", "sheep", "cow", "horse",
}


class MissionPlan(BaseModel):
    """A complete, self-contained mission. Same JSON in -> same flight out."""
    action: Action
    route: Optional[str] = Field(None)
    waypoints: List[WaypointNED] = Field(default_factory=list, max_length=50)
    altitude_m: float = Field(10.0)
    speed_ms: float = Field(5.0)
    loops: int = Field(1)
    return_to_launch: bool = Field(True)

    # follow_target fields
    target_class: Optional[str] = Field(None)
    follow_duration_s: float = Field(30.0)
    search_yaw_rate_deg_s: float = Field(20.0)

    reasoning: str = Field("", max_length=500)

    @model_validator(mode="after")
    def action_requirements(self):
        if self.action == Action.PATROL_ROUTE and not self.route:
            raise ValueError("patrol_route requires a 'route' name")
        if self.action == Action.FLY_WAYPOINTS and len(self.waypoints) < 2:
            raise ValueError("fly_waypoints requires at least 2 waypoints")
        if self.action == Action.FOLLOW_TARGET and not self.target_class:
            raise ValueError("follow_target requires a 'target_class'")
        return self


def schema_for_prompt() -> str:
    import json
    return json.dumps(MissionPlan.model_json_schema(), indent=2)
