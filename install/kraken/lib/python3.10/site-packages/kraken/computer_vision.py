import rclpy
from rclpy.node import Node

class ComputerVision(Node):

    def __init__(self):
        super().__init__('computer_vision')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Computer Vision Running')


def main(args=None):
    rclpy.init(args=args)

    computer_vision = ComputerVision()

    rclpy.spin(computer_vision)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    computer_vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
