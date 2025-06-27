import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64

sys.path.append("/home/vboxuser/kraken-nano/ROS/ws/src/kraken/kraken/include")

from simulation import Simulation

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sim = Simulation(self)
        self.sim.left(10)

    def timer_callback(self):
        self.get_logger().info('Controller Running')
        altimeter = self.sim.get_altimeter()
        if altimeter is not None:
	        self.get_logger().info(str(altimeter.vertical_position))


def main(args=None):
    rclpy.init(args=args)

    controller = Controller() 

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
