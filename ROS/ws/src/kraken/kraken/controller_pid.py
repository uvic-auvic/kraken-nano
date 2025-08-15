import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64, String
import time
import json

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

#from simulation import Simulation
from motorboard import MotorBoard
from pid import PID
from serial import Serial

from custom.msg import PoseE

class ControllerPID(Node):

        def __init__(self):
                super().__init__('controllerpid')
                
                self.subscription = self.create_subscription(PoseE, '/state_estimator/pose', self.pose_callback, 10)
                
                # Subscribe to motor commands from planner
                self.motor_command_subscriber = self.create_subscription(
                    String, '/motor/command', self.motor_command_callback, 10)
                
                pid_period = 0.01  # seconds
                self.pid_timer = self.create_timer(pid_period, self.pid_callback)
                self.logger = self.get_logger()
                
                #self.sim = Simulation(self)
                
                #self.kill_switch = Serial("/dev/ttyTCU0", 115200, timeout=3)
                
                # Initialize motor board
                try:
                    self.mb = MotorBoard("/dev/ttyTHS1")
                    self.logger.info("Motor board initialized")
                    #self.mb.init_motors()
                    time.sleep(1)
                except Exception as e:
                    self.logger.error(f"Failed to initialize motor board: {str(e)}")
                    self.mb = None
                
                # Motor control state
                self.current_command = None
                self.command_start_time = None
                
                self.pose = None
                
                self.logger.info("Controller initialized and ready for planner commands")

        def motor_command_callback(self, msg):
                """Callback for motor commands from planner"""
                try:
                    command_data = json.loads(msg.data)
                    command = command_data.get('command', '')
                    speed = command_data.get('speed', 15)
                    
                    self.logger.info(f"Received motor command: {command} at speed {speed}")
                    
                    if self.mb:
                        self.execute_motor_command(command, speed)
                    else:
                        self.logger.warn("Motor board not available, simulating command")
                        
                except json.JSONDecodeError as e:
                    self.logger.error(f"Error parsing motor command: {str(e)}")

        def execute_motor_command(self, command: str, speed: int):
                """Execute a motor command"""
                if not self.mb:
                    return
                    
                try:
                    # Reset motor masks
                    self.mb.positive_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                    self.mb.negative_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                    
                    # Execute specific command
                    if command == "forward":
                        self.mb.forward()
                        
                    elif command == "backward":
                        self.mb.backward()
                        
                    elif command == "left":
                        self.mb.left()
                        
                    elif command == "right":
                        self.mb.right()
                        
                    elif command == "up":
                        self.mb.up()
                        
                    elif command == "down":
                        self.mb.down()
                        
                    elif command == "yaw_ccw":
                        self.mb.yaw_ccw()
                        
                    elif command == "yaw_cw":
                        self.mb.yaw_cw()
                        
                    elif command == "stop":
                        self.mb.cut_motors()
                        self.logger.info("Motors stopped")
                        return
                        
                    else:
                        self.logger.warn(f"Unknown motor command: {command}")
                        return
                    
                    # Send motors with specified speed
                    if speed > 0:
                        self.mb.send_motors(min(speed, 127))
                        self.logger.debug(f"Executed {command} at speed {speed}")
                    
                except Exception as e:
                    self.logger.error(f"Error executing motor command {command}: {str(e)}")

        def emergency_stop(self):
                """Emergency stop all motors"""
                if self.mb:
                    self.mb.cut_motors()
                self.logger.info("EMERGENCY STOP executed")

        
        def pid_callback(self):
                if self.pose:
                        forward_speed = self.forward_pid.calculate(self.pose.pos.x)
                        up_speed = self.up_pid.calculate(self.pose.pos.z)
                        left_speed = self.left_pid.calculate(self.pose.pos.y)
                        yaw_speed = self.yaw_pid.calculate(self.pose.rot.yaw)
                        
                        self.sim.forward(forward_speed)
                        self.sim.up(forward_speed)
                        self.sim.left(left_speed)
                        self.sim.yaw(yaw_speed)
                        
                        #self.logger.info(str(self.pose.rot))
        
                
	        
	        
        def pose_callback(self, msg):
                """Callback for pose updates"""
import pyrealsense2 as rs
import numpy as np


def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

p = initialize_camera()
try:
    while True:
        f = p.wait_for_frames()
        accel = accel_data(f[0].as_motion_frame().get_motion_data())
        gyro = gyro_data(f[1].as_motion_frame().get_motion_data())
        print("accelerometer: ", accel)
        print("gyro: ", gyro)

finally:
    p.stop()
                
                self.pose = msg


def main(args=None):
        rclpy.init(args=args)

        controllerpid = ControllerPID() 

        try:
            rclpy.spin(controllerpid)
        except KeyboardInterrupt:
            controllerpid.logger.info("Controller shutting down...")
            controllerpid.emergency_stop()
        finally:
            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
        main()
