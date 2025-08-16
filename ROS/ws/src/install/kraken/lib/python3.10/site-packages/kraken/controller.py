import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64, String, Int32, Float32
from std_msgs.msg import Int32
import time
import json
import signal

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

# from simulation import Simulation
from motorboard import MotorBoard
from pid import PID
from serial import Serial

from custom.msg import PoseE

"""
init:
    sub to pose and task
    create depth and yaw pid timers
    keep track of target/current yaw and depth

when task received:
    get command, speed/angle, duration
    if command is depth and yaw
        set class variable
    execute command at speed

yaw timer:
    continuous pid on current/target yaw
    send reset after task to SE
    send confirmation
    pause if going left or right

depth timer:
    continuous pid on current/target depth
    send confirmation
"""


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Subscribe to pose from state estimator
        self.subscription = self.create_subscription(PoseE, '/state_estimator/pose', self.pose_callback, 10)

        # Subscribe to motor commands from planner
        #self.motor_command_subscriber = self.create_subscription(
            #String, '/motor/command', self.motor_command_callback, 10)

        signal.signal(signal.SIGINT, self.signal_handler)

        depth_period = 0.1  # seconds
        self.depth_timer = self.create_timer(depth_period, self.depth_callback)

        yaw_period = 0.1  # seconds
        self.yaw_timer = self.create_timer(yaw_period, self.yaw_callback)

        self.logger = self.get_logger()
        self.reset_yaw_pub =self.create_publisher(Float32, "/controller/reset_yaw", 10)
        self.depth_target = 0.9
        self.depth_current = 0.0
        self.yaw_target = 0.0
        self.yaw_current = 0.0
        self.yaw_correction = True
        self.depth_correction = True

        self.states = [
            self.state1,
            self.state2,
        ]

        self.current_state = 0

        # Initialize motor board
        try:
            self.mb = MotorBoard("/dev/ttyTHS1")
            self.logger.info("Motor board initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize motor board: {str(e)}")
            self.mb = None

        # Motor control state
        self.current_command = None
        self.command_start_time = None

        self.logger.info("Controller initialized and ready for planner commands")

        self.run_next_state()

    def run_next_state(self):
        if self.current_state < len(self.states):
            func = self.states[self.current_state]
            func()
            self.current_state += 1
            self.run_next_state()

    def non_blocking_delay(self, duration_seconds):
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration_seconds)
        while self.get_clock().now() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)

    def state1(self):
        self.non_blocking_delay(10.0)
        self.logger.info("Running state 1")
        self.execute("forward", 80)
        self.non_blocking_delay(8.0)
        self.execute("stop")

    def state2(self):
        self.logger.info("Running state 2")
        self.execute("left", 30)
        self.non_blocking_delay(2.0)
        self.execute("stop")   

    def initialize(self):
        self.mb.init_motors()
        self.yaw_correction = True
        self.depth_correction = True

    def execute(self, command: str, magnitude: int = 0):
        """Callback for motor commands from planner"""
        self.logger.info(f"Received motor command: {command} at speed {magnitude}")

        if command == "setdepth":
            self.depth_correction = True
            self.depth_target = magnitude
            return

        elif command == "setyaw":
            self.yaw_correction = True
            self.yaw_target = magnitude
            return
        
        elif command == "stop":
            self.yaw_target = self.yaw_current
            self.mb.cut_motors()
            self.yaw_correction = True
            return

        elif command == "stopall":
            if self.mb:
                self.yaw_correction = False
                self.depth_correction = False
                self.mb.cut_motors()
            return

        if command == "forward":
            self.mb.forward()

        elif command == "backward":
            self.mb.backward()

        elif command == "left":
            self.yaw_correction = False
            self.mb.left()

        elif command == "right":
            self.yaw_correction = False
            self.mb.right()
        
        else:
            self.logger.error(f"Unknown command: {command}")

        if magnitude > 0:
            self.mb.send_motors(min(magnitude, 127))
            self.logger.debug(f"Executed {command} at speed {magnitude}")

    def yaw_callback(self):
        self.logger.info("Yaw timer running")

        if not self.yaw_correction:
            return

        yaw_K = 80

        self.logger.info(f"Yaw: {self.yaw_current}, {self.yaw_target}")
        
        if self.yaw_current > self.yaw_target:
            self.mb.yaw_ccw()
        else:
            self.mb.yaw_cw()
        speed = min(int(abs(self.yaw_current - self.yaw_target)*yaw_K), 127)
        self.mb.send_motors(speed)
        
        if self.yaw_target != 0 and abs(self.yaw_target - self.yaw_current) < 0.005:
            reset_msg = Float32()
            reset_msg.data = 0.0
            self.yaw_target = 0
            self.reset_yaw_pub.publish(reset_msg)

    def depth_callback(self):
        self.logger.info("Depth")

        if not self.depth_correction:
            return

        down_K = 80
        up_K = 60

        self.logger.info(f"Depth: {self.depth_current}, {self.depth_target}")
        
        if self.depth_current > self.depth_target:
            self.mb.up()
            speed = min(int(abs(self.depth_current - self.depth_target)*up_K), 127)
        else:
            self.mb.down()
            speed = min(int(abs(self.depth_current - self.depth_target)*down_K), 127)
        self.mb.send_motors(speed)

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
        # print(msg)
        if msg.rot.yaw:
            self.yaw_current = msg.rot.yaw
        if msg.pos.z:
            self.depth_current = msg.pos.z

    def delay(self, seconds):
        current = time.time()
        while time.time() - current < seconds:
            continue

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        self.execute("stopall")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    """
    controller.mb.forward()
    controller.mb.send_motors(30)
    controller.delay(3)
    #time.sleep(3)
    controller.mb.cut_motors()
    controller.mb.left()
    controller.mb.send_motors(30)
    controller.delay(3)
    #time.sleep(3)
    controller.mb.cut_motors()
    """


    #controller.delay(4)

    #controller.execute("setdepth", 0)

    #print("test")
    
    #self.mb.init_motors()
    #controller.mb.forward()
    #controller.mb.send_motors(30)
    #time.sleep(3)
    #controller.mb.cut_motors()

    #controller.initialize()
    #time.sleep(2)
    #controller.execute("forward", 20)
    #time.sleep(1)
    #controller.execute("stop")

    #controller.step1()
    #controller.step2()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.logger.info("Controller shutting down...")
        #controller.emergency_stop()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
