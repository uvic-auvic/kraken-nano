import rclpy
from rclpy.node import Node
import sys
import time
from scipy.spatial.transform import Rotation

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
        self.euler = None
        self.current = time.time()
        self.prev = time.time()

    def timer_callback(self):
        depth = self.sim.get_depth()
        orient = self.sim.get_orientation()
        
        if orient is not None:
                quat = (orient.x, orient.y, orient.z, orient.w)
                
                rot = Rotation.from_quat(quat)
                self.euler = rot.as_euler('xyz')

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
        self.get_logger().info(str(orient))
    
    """
    Estimated movement of the sub:
    
    X drag force: 40 * v^2 N
    Y drag force: 80 * v^2 N
    Z drag force: 200 * v^2 N
    Maximum forward force per thruster: 23.14 N
    Maximum backward force per thruster: 18.14 N
    AUV mass: 40kg
    
    Expected values:
        Forward acceleration: 1.157 - v^2
        Backward acceleration: 0.907 - v^2
        Right acceleration: 0.907 - 2 * v^2
        Left acceleration: 1.157 - 2 * v^2
        Up acceleration: 2.314 - 5 * v^2
        Down acceleration: 1.814 - 5 * v^2
    
    """


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
