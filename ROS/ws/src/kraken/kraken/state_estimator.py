import rclpy
from rclpy.node import Node
import sys
import time
import serial
import json

from custom.msg import PoseE
from std_msgs.msg import String

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_pub = self.create_publisher(PoseE, "/state_estimator/pose", 10)
        self.imu_sub = self.create_subscription(String, "gyro_data", self.imu_callback, 10)
        
        self.logger = self.get_logger()

        self.depth_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=3)
        
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

        depth = self.get_depth()
        
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
                
        if self.yaw_velocity is not None:
            # self.logger.info(str(self.yaw_velocity))
            self.yaw += delta * self.yaw_velocity
            msg.rot.yaw = self.yaw

        self.pose_pub.publish(msg)
        self.logger.info(f"Published pose: {msg.pos.x}, {msg.pos.y}, {msg.pos.z}, {msg.rot.yaw}, {msg.rot.roll}, {msg.rot.pitch}")

    def get_depth(self):
        try:
            depth_str = ""
            depth_byte = self.depth_serial.read()
            while depth_byte != b"\n":
                depth_byte = self.depth_serial.read()

            while depth_byte != b"\r":
                depth_byte = self.depth_serial.read()
                depth_str += depth_byte.decode()
                
            return float(depth_str)
        except (ValueError, serial.SerialTimeoutException):
            return None

    def imu_callback(self, msg):
        data = json.loads(str(msg.data))
        self.yaw_velocity = float(data["gyro"]["y"])

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
