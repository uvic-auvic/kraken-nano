import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64, String, Int32
from std_msgs.msg import Int32
import time
import json

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

# from simulation import Simulation
from motorboard import MotorBoard
from pid import PID
from serial import Serial

from custom.msg import PoseE


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.subscription = self.create_subscription(PoseE, '/state_estimator/pose', self.pose_callback, 10)

        # Subscribe to motor commands from planner
        self.motor_command_subscriber = self.create_subscription(
            String, '/motor/command', self.motor_command_callback, 10)

        pid_period = 0.1  # seconds
        self.pid_timer = self.create_timer(pid_period, self.pid_cont)
        self.logger = self.get_logger()
        self.reset_yaw_pub =self.create_publisher(Float64, "/controller/reset_yaw", 10)
        self.depth_reached_pub = self.create_publisher(Float64, "/controller/depth_reached", 10)
        self.set_depth = 0;
        #self.sim = Simulation(self)

        self.command_status_pub = self.create_publisher(Int32, "/controller/command_status", 10)
        self.command_stat = 0; # sends 0 for none, 1 for received, 2 for complete, 3 for overwritten
        # self.kill_switch = Serial("/dev/ttyTCU0", 115200, timeout=3)

        # Initialize motor board
        try:
            self.mb = MotorBoard("/dev/ttyTHS1")
            self.logger.info("Motor board initialized")
            # self.mb.init_motors()
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
            command = command_data["command"]
            speed = command_data["speed"]
            duration = command_data.get('duration', '')

            self.logger.info(f"Received motor command: {command} at speed {speed}")
            if self.command_stat == 1 or self.command_stat == 3:
                self.command_stat = 3
            else:
                self.command_stat = 1
            
            # Create and publish Int32 message
            status_msg = Int32()
            status_msg.data = self.command_stat
            self.command_status_pub.publish(status_msg)

            if self.mb:
                self.execute_motor_command(command, speed, duration)
            else:
                self.logger.warn("Motor board not available, simulating command")
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Error parsing motor command: {str(e)}")

    def execute_motor_command(self, command: str, speed: int, duration: int):
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

            elif command == "setdepth":
                self.set_depth = speed
                return

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

            time_start = time.time()
            while time.time() < time_start + duration or self.command_stat == 2:
                pass
            self.command_stat = 2
            
            # Create and publish Int32 message
            status_msg = Int32()
            status_msg.data = self.command_stat
            self.command_status_pub.publish(status_msg)

        except Exception as e:
            self.logger.error(f"Error executing motor command {command}: {str(e)}")

    def emergency_stop(self):
        """Emergency stop all motors"""
        if self.mb:
            self.mb.cut_motors()
        self.logger.info("EMERGENCY STOP executed")

    def yaw_pid(self):
        if not self.pose:
            return
            
        yaw = self.pose.yaw
        yaw_K = 30
        
        reset_msg = Float32()
        if self.current_command == "yaw_ccw" or self.current_command == "yaw_cw":
            reset_msg.data = 1.0
            self.reset_yaw_pub.publish(reset_msg)
            return
        elif self.current_command == "left" or self.current_command == "right":
            pass
        else:
            if self.mb:
                if yaw > 0:
                    self.mb.yaw_cw()
                elif yaw < 0:
                    self.mb.yaw_ccw()
                speed = min(int(abs(yaw)*yaw_K), 127)
                self.mb.send_motors(speed)
        
        reset_msg.data = 0.0
        self.reset_yaw_pub.publish(reset_msg)    

    def depth_pid(self):
        if not self.pose:
            return
            
        depth = self.pose.pos.z
        down_K = 60
        up_K = 20
        
        depth_msg = Float32()
        if self.set_depth - 0.2 < depth < self.set_depth + 0.2:
            depth_msg.data = 1.0
            self.depth_reached_pub.publish(depth_msg)
            return
        elif depth > self.set_depth and self.mb:
            self.mb.up()
            speed = min(int(abs(depth - self.set_depth) * up_K), 127)
            self.mb.send_motors(speed)
        elif depth < self.set_depth and self.mb:
            self.mb.down()
            speed = min(int(abs(depth-self.set_depth)*down_K), 127)
            self.mb.send_motors(speed)
        
        depth_msg.data = 0.0
        self.depth_reached_pub.publish(depth_msg)

    def pid_cont(self):
        if self.pose:
            self.yaw_pid()
            self.depth_pid()

    """
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
    """

    def pose_callback(self, msg):
        """Callback for pose updates"""
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.logger.info("Controller shutting down...")
        controller.emergency_stop()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()