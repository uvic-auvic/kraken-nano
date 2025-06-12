from messages import (
    make_task_message, TaskType, Direction,
    make_search_space_message, OBJECT_TYPES, DetectedType
)
from publisher import PlannerPublisher
from typing import Optional

class PlannerNode:
    def __init__(self, publisher: PlannerPublisher):
        self.publisher = publisher
        self.search_space = {"objects": []}
        # last vision detection (black‐box): None or DetectedType
        self._last_detection: Optional[DetectedType] = None
        # track whether gate is centered in FOV
        self._gate_centered: bool = False
        # track whether slalom gate is centered in FOV
        self._slalom_centered: bool = False

    def update_search_space(self, objects):
        # Example objects: ['gate', 'shark']
        self.search_space = make_search_space_message(objects)
        self.publisher.publish_search_space(self.search_space)

    def decide_and_publish_task(self):
        # Example logic: If 'gate' in search space, move forward, else yaw search
        objects = self.search_space.get("objects", [])
        if "gate" in objects:
            task = make_task_message(TaskType.MOVE, Direction.X, 2.0)
        else:
            task = make_task_message(TaskType.MOVE, Direction.YAW, 15.0)
        self.publisher.publish_task(task)

    def publish_move(self, direction: Direction, magnitude: float):
        task = make_task_message(TaskType.MOVE, direction, magnitude)
        self.publisher.publish_task(task)

    def publish_dropper(self, state: Direction):
        task = make_task_message(TaskType.DROPPER, state)
        self.publisher.publish_task(task)

    def publish_grabber(self, state: Direction):
        task = make_task_message(TaskType.GRABBER, state)
        self.publisher.publish_task(task)

    def publish_torpedo(self, direction: Optional[Direction] = None, magnitude: Optional[float] = None):
        task = make_task_message(TaskType.TORPEDO, direction, magnitude)
        self.publisher.publish_task(task)

    def on_detection(self, detection: DetectedType):
        """
        Should be called whenever vision pipeline reports a new detection.
        """
        self._last_detection = detection

    def get_last_detection(self) -> Optional[DetectedType]:
        """Return the last seen detection (or None)."""
        return self._last_detection

    def on_gate_centered(self, centered: bool):
        """
        Should be called when vision pipeline reports whether the gate is centered in the camera FOV.
        """
        self._gate_centered = centered

    def on_slalom_centered(self, centered: bool):
        """
        Should be called when vision pipeline reports whether the slalom gate is centered in the camera FOV.
        """
        self._slalom_centered = centered

    def is_gate_centered(self) -> bool:
        """
        Returns True if the gate is currently deemed centered in the camera FOV.
        """
        return self._gate_centered

    def is_slalom_centered(self) -> bool:
        """
        Returns True if the slalom gate is currently centered in the camera FOV.
        """
        return self._slalom_centered

# Example send function for testing (prints messages)
def dummy_send_func(topic, message):
    print(f"Publishing to {topic}: {message}")

if __name__ == "__main__":
    publisher = PlannerPublisher(dummy_send_func)
    node = PlannerNode(publisher)
    # Simulate a perception update
    node.update_search_space(['shark'])
    node.decide_and_publish_task()
    node.update_search_space(['gate'])
    node.decide_and_publish_task()
