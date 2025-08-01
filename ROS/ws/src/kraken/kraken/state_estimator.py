import rclpy
from rclpy.node import Node
import sys
import time
from scipy.spatial.transform import Rotation

from custom.msg import PoseE

sys.path.append("/home/ubuntu/Documents/uvic/kraken-nano/ROS/ws/src/kraken/kraken/include")

from simulation import Simulation

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_pub = self.create_publisher(PoseE, "/state_estimator/pose", 10)
        
        self.sim = Simulation(self)
        self.logger = self.get_logger()
        
        # Position
        self.x = 0
        self.y = 0
        self.z = 0
        
        # Velocity
        self.u = 0
        self.v = 0
        self.w = 0
        self.euler = None
        self.current = time.time()
        self.prev = time.time()

    def timer_callback(self):        
        
        self.current = time.time()
        delta = self.current - self.prev
        self.prev = self.current
        accel = self.sim.get_acceleration()
        depth = self.sim.get_depth()
        orient = self.sim.get_orientation()
        
        msg = PoseE()
        msg.pos.x = 0.0
        msg.pos.y = 0.0
        msg.pos.z = 0.0
        msg.rot.yaw = 0.0
        msg.rot.roll = 0.0
        msg.rot.pitch = 0.0
        
        if depth is not None:
                self.z = depth
                msg.pos.z = float(self.z)
        if accel is not None:
                self.u += delta * accel.x
                self.v += delta * accel.y
                
                self.x += delta * self.u
                self.y += delta * self.v
                
                msg.pos.x = float(self.x)
                msg.pos.y = float(self.y)
                
        if orient is not None:
                quat = (orient.x, orient.y, orient.z, orient.w)
                
                rot = Rotation.from_quat(quat)
                self.euler = rot.as_euler('zyx')
                
                msg.rot.yaw = float(self.euler[0])
                msg.rot.roll = float(self.euler[2])
                msg.rot.pitch = float(self.euler[1])
    
    
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
