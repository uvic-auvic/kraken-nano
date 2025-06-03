import rclpy
from rclpy.node import Node

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('State Estimator Running')


def main(args=None):
    rclpy.init(args=args)

    state_estimator = StateEstimator()

    rclpy.spin(state_estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
