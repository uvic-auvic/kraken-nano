from enum import Enum
from typing import Optional, List, Dict

class TaskType(str, Enum):
    MOVE = "move"
    DROPPER = "dropper"
    GRABBER = "grabber"
    TORPEDO = "torpedo"

class Direction(str, Enum):
    # For move
    X = "x"
    Y = "y"
    Z = "z"
    YAW = "yaw"
    ROLL = "roll"
    PITCH = "pitch"
    # For dropper/grabber
    ON = "on"
    OFF = "off"

def make_task_message(
    task_type: TaskType,
    direction: Optional[Direction] = None,
    magnitude: Optional[float] = None
) -> dict:
    msg = {
        "type": task_type.value,
    }
    if direction:
        msg["direction"] = direction.value
    if magnitude is not None:
        msg["magnitude"] = magnitude
    return msg

OBJECT_TYPES = [
    "none", "gate", "sawfish", "shark",
    "slalom_out", "slalom_in", "bin",
    "torpedo_pic", "torpedo_oct", "torpedo_circle",
    "torpedo_square", "octagon", "table", "cup", "spoon"
]

def make_search_space_message(objects: List[str]) -> dict:
    return {"objects": objects}

class DetectedType(str, Enum):
    GATE    = "gate"
    PICTURE = "picture"
    SLALOM  = "slalom"
    ORANGE_PATH = "orange_path"
    BIN = "bin"
    RED_SQUARE = "red_square"
