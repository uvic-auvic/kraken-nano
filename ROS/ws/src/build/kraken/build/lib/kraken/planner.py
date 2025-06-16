import rclpy
from rclpy.node import Node

from custom.msg import Task

class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.task_publisher = self.create_publisher(Task, '/planner/task', 10)

    def timer_callback(self):
        self.get_logger().info('Planner Running')
        task = Task()
        task.type = "move"
        task.direction = "x"
        task.magnitude = 2
        self.task_publisher.publish(task)


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
