from messages import (
    make_task_message, TaskType, Direction,
    make_search_space_message, OBJECT_TYPES
)
from publisher import PlannerPublisher

class PlannerNode:
    def __init__(self, publisher: PlannerPublisher):
        self.publisher = publisher
        self.search_space = {"objects": []}

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
