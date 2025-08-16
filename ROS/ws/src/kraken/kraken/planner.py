import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from enum import Enum

class TaskType(Enum):
    MOVE_FORWARD = "forward"
    MOVE_BACKWARD = "backward"
    MOVE_LEFT = "left"
    MOVE_RIGHT = "right"
    ROTATE_LEFT = "yaw_ccw"
    ROTATE_RIGHT = "yaw_cw"
    SET_DEPTH = "setdepth"
    STOP_MOTORS = "stop"

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.objects = [0] * 8
        self.detection_info = {}

        # Create publisher for motor commands
        self.motor_command_publisher = self.create_publisher(String, '/motor/command', 10)
        
        # Task management
        self.current_task = None
        self.task_queue = []
        self.task_start_time = None
        
        # Timer for task execution
        self.task_timer = self.create_timer(0.5, self.execute_current_task)  # 2Hz
        
        self.get_logger().info('Planner Node initialized - Basic movement tasks only')

        #subscribe to compVision objects
        self.create_subscription(String, 'objects', self.compVision_callback, 10)

        #subscribe to the compVision detectionInfo
        self.create_subscription(String, 'detection_info', self.detectionInfo_callback, 10)

    def compVision_callback(self, msg):
        self.objects = msg.data
    
    def detectionInfo_callback(self, msg):
        self.detection_info = json.loads(msg.data)

    def send_motor_command(self, command: str, speed: int, duration: float):
        """Send motor command to controller"""
        motor_msg = {
            'command': command,
            'speed': speed,
            'duration': duration
        }
        
        message = String()
        message.data = json.dumps(motor_msg)
        self.motor_command_publisher.publish(message)

    # Convenience methods for basic movements

    #FRONT FACING MOTOR FUCNTIONS
    def move_forward(self, duration: float = 3.0, speed: int = 60):
        self.send_motor_command("forward", speed, duration)

    def move_backward(self, duration: float = 3.0, speed: int = 60):
        self.send_motor_command("backward", speed, duration)

    def stop_front_facing_motors(self):
        self.send_motor_command("backward", speed, duration)

    #HORIZONTAL FACING MOTOR FUCNTIONS
    def move_left(self, duration: float = 2.0, speed: int = 50):
        self.send_motor_command("left", speed, duration)

    def move_right(self, duration: float = 2.0, speed: int = 50):
        self.send_motor_command("right", speed, duration)

    def rotate_left(self, duration: float = 2.0, speed: int = 40):
        self.send_motor_command("yaw_ccw", speed, duration)

    def rotate_right(self, duration: float = 2.0, speed: int = 40):
        self.send_motor_command("yaw_cw", speed, duration)
    
    def stop_horizontal_motors(self):
        self.send_motor_command("right", speed, duration)

    #STOP COMMAND
    def stop_all_motors(self):
        """Immediately stop all motors"""
        self.send_motor_command("stop", 0, 0)

    #VERTICAL FACING MOTOR FUNCTIONS
    def set_depth(self, depth: float):
        """Set target depth"""
        self.send_motor_command("setdepth", depth, 0)


def main(args=None):
    rclpy.init(args=args)
    
    planner = Planner()
    
    try:
        planner.get_logger().info("Starting planner test scenarios...")

        planner.move_forward(duration=5, speed=80)
        planner.stop_forward()
        planner.set_depth(depth=0.75)
        #planner.rotate_left(duration=6, speed=15)
        #planner.move_forward(duration=10, speed=120)
        planner.rotate_left(duration=2, speed=30)
        planner.move_forward(duration=8.0, speed=20)  # Move up to start
        # planner.move_backward(duration=1.0, speed=20)
        # planner.move_left(duration=1.0, speed=20)
        planner.move_right(duration=2, speed=70)
        # planner.stop_all_motors()
        planner.move_forward(duration=8, speed=120)
        # time.sleep(5.0)
        planner.stop_all_motors()
        # planner.rotate_left(duration=2.0, speed=20)
        #wait a bit
        #time_start = time.time()
        #while time.time() < time_start + 5:
            #print("Waiting for 5 seconds before next action...")

        # planner.rotate_right(duration=1.0, speed=20)
        #planner.set_depth(depth=2)
        # Set depth to 1 meter


        
        # Execute all steps
        # planner.step1_go_to_depth()
        # planner.step2_yaw_rotation()
        # planner.step3_center_gate()
        # planner.step4_move_through_gate()
        # planner.step5_continue_forward()
        # planner.step6_find_slalom()
        # planner.step7_center_slalom_red()
        # planner.step8_slalom_distance()
        # planner.step9_position_slalom_right()
        # planner.step10_full_gas()
        
    except KeyboardInterrupt:
        planner.get_logger().info('Planner shutting down...')
        planner.stop_all_motors()  # Stop motors on shutdown
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
