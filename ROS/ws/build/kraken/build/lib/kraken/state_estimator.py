import rclpy
from rclpy.node import Node
import sys
import time
import serial
import json

from custom.msg import PoseE

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

import ms5837
from simulation import Simulation

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_pub = self.create_publisher(PoseE, "/state_estimator/pose", 10)
        self.imu_sub = self.create_subscription(String, "gyro_data", self.imu_callback,10)
        
        self.sim = Simulation(self)
        self.logger = self.get_logger()

        self.depth_serial = serial.Serial("/dev/ttyUSB0", 115200, 3)
        
        # Position
        self.z = 0
        self.yaw = 0
        
        # Velocity
        self.yaw_velocity = 0
        self.current = time.time()
        self.prev = time.time()

    def timer_callback(self):        
        
        self.current = time.time()
        delta = self.current - self.prev
        self.prev = self.current

        depth = get_depth()
        
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
                
        if yaw_velocity is not None:
                self.yaw += delta * self.yaw_velocity

        self.pose_pub.publish(msg)

    def get_depth(self):
        depth_str = self.depth_serial.readline()
        return float(depth_str[:-2])

    def imu_callback(self, msg):
        data = json.loads(msg)
        self.yaw_velocity = int(msg["gyro"]["y"])

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
