import json

class PlannerPublisher:
    def __init__(self, send_func):
        self.send_func = send_func  # Can be ROS publisher, socket, etc.

    def publish_task(self, task_msg: dict):
        # Publishes to /planner/task
        message = json.dumps(task_msg)
        self.send_func("/planner/task", message)

    def publish_search_space(self, search_space_msg: dict):
        # Publishes to /planner/search_space
        message = json.dumps(search_space_msg)
        self.send_func("/planner/search_space", message)
