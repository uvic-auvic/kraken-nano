import rclpy
from rclpy.node import Node
import sys
import time

sys.path.append("/home/vboxuser/kraken-nano/ROS/ws/src/kraken/kraken/include")

from simulation import Simulation

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sim = Simulation(self)
        self.x = 0
        self.y = 0
        self.z = 0
        self.current = time.time()
        self.prev = time.time()

    def timer_callback(self):
        depth = self.sim.get_depth()
        orient = self.sim.get_orientation()

        self.current = time.time()
        delta = self.current - self.prev
        self.prev = time.time()
        accel = self.sim.get_acceleration()
        
        if depth is not None:
                self.z = depth
        if accel is not None:
                self.x += (delta ** 2 / 2) * accel.x
                self.y += (delta ** 2 / 2) * accel.y
                
        self.get_logger().info(str(self.x) + ', ' + str(self.y) + ', ' + str(self.z))


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
