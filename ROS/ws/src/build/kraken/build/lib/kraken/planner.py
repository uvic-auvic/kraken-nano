import rclpy
from rclpy.node import Node

class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Planner Running')


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
