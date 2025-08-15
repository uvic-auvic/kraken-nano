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


    def add_task(self, task_type: TaskType, parameters: dict = None):
        """Add a task to the queue"""
        task = {
            'task_type': task_type.value,
            'parameters': parameters or {},
            'timestamp': self.get_clock().now().to_msg()
        }
        self.task_queue.append(task)
        self.get_logger().info(f'Added task: {task_type.value}')

    def compVision_callback(self, msg):
        self.objects = msg.data
    
    def detectionInfo_callback(self, msg):
        self.detection_info = json.loads(msg.data)

    def execute_current_task(self):
        """Execute the current task or get next task from queue"""
        # If no current task, get next from queue
        if not self.current_task and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            self.task_start_time = time.time()
            self.get_logger().info(f'Starting task: {self.current_task["task_type"]}')
            
        # Execute current task
        if self.current_task:
            self.execute_task_step()

 

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
    def move_forward(self, duration: float = 3.0, speed: int = 60):
        """Add move forward task"""
        self.send_motor_command("forward", speed, duration)

    def move_backward(self, duration: float = 3.0, speed: int = 60):
        """Add move backward task"""
        self.send_motor_command("backward", speed, duration)


    def move_left(self, duration: float = 2.0, speed: int = 50):
        """Add move left task"""
        self.send_motor_command("left", speed, duration)

    def move_right(self, duration: float = 2.0, speed: int = 50):
        """Add move right task"""
        self.send_motor_command("right", speed, duration)

    def rotate_left(self, duration: float = 2.0, speed: int = 40):
        """Add rotate left task"""
        self.send_motor_command("yaw_ccw", speed, duration)

    def rotate_right(self, duration: float = 2.0, speed: int = 40):
        """Add rotate right task"""
        self.send_motor_command("yaw_cw", speed, duration)


    def stop_all_motors(self):
        """Immediately stop all motors"""
        self.send_motor_command("stop", 0, 0)


    def set_depth(self, depth: float):
        """Set target depth"""
        self.send_motor_command("setdepth", depth, 0)

    def step1_go_to_depth(self):
        """Step 1: Go to Depth"""
        self.get_logger().info("SENDING SETDEPTH")
        self.send_motor_command(command="setdepth", speed=1, duration=0)
        self.get_logger().info("Step 1: Go to Depth")
        self.set_depth(depth=1)
        time.sleep(0.1)
        self.get_logger().info("Step 1: Completed")

    def step2_yaw_rotation(self):
        """Step 2: Yaw rotation"""
        isGateFound = False
        while not isGateFound:
            if self.objects[2] == 1:
                isGateFound = True
                break
            self.get_logger().info("Step 2: Yaw Rotation")
            self.rotate_left(duration=1.0, speed=100)
            time.sleep(1.0)

    def step3_center_gate(self):
        """Step 3: Yaw slower till gate centered"""
        isGateCentered = False
        while not isGateCentered:
            if self.detection_info.get('center_x'):
                if self.detection_info.get('class_name') == 'Full Gate':
                    center_x = self.detection_info['center_x']
                    if center_x > 400:
                        self.rotate_right(duration=0.5, speed=50)
                    elif center_x < 240:
                        self.rotate_left(duration=0.5, speed=50)

            if center_x > 240 and center_x < 400:
                self.get_logger().info("Gate is centered")
                isGateCentered = True

    def step4_move_through_gate(self):
        """Step 4: gate centered go go go"""
        isGateFound = True
        while isGateFound:
            if self.objects[2] == 0:
                isGateFound = False
            self.move_forward(duration=0.5, speed=127)  # Move through gate

    def step5_continue_forward(self):
        """Step 5: gate centered go go go"""
        self.move_forward(duration=2, speed=127)  # Move through gate

    def step6_find_slalom(self):
        """Step 6: find the slalom"""
        isSlalomFound = False
        gatePayload = None
        while not isSlalomFound:
            if self.objects[3] == 1:
                isSlalomFound = True
            self.get_logger().info("Step 6: Searching for Slalom")
            self.rotate_left(duration=0.5, speed=100)

    def step7_center_slalom_red(self):
        """Step 7: is Slalom Red Centered"""
        isSlalomRedCentered = False
        while not isSlalomRedCentered:
            if self.detection_info.get('center_x'):
                if self.detection_info.get('class_name') == 'Red Slalom':
                    center_x = self.detection_info['center_x']
                    if center_x > 400:
                        self.rotate_right(duration=0.5, speed=50)
                    elif center_x < 240:
                        self.rotate_left(duration=0.5, speed=50)

            if center_x > 240 and center_x < 400:
                self.get_logger().info("Gate is centered")
                isGateCentered = True

    def step8_slalom_distance(self):
        """Step 8: get slalom Red distance to 2 meters away"""
        isSlalomRedTwoMetersAway = False
        while not isSlalomRedTwoMetersAway:
            if self.detection_info.get('distance'):
                if self.detection_info.get('class_name') == 'Red Slalom':
                    distance = self.detection_info['distance']
                    if distance > 2.0:
                        self.move_forward(duration=0.5, speed=50)
                    else:
                        self.get_logger().info("Slalom Red is 2 meters away")
                        isSlalomRedTwoMetersAway = True

    def step9_position_slalom_right(self):
        """Step 9: get slalom Red into the far right third of image"""
        isInRightThird = False
        while not isInRightThird:
            if self.detection_info.get('center_x'):
                center_x = self.detection_info['center_x']
                if self.detection_info.get('class_name') == 'Red Slalom':
                    self.move_right(duration=0.5, speed=50)
            if center_x > 480:
                self.get_logger().info("Gate is centered")
                isInRightThird = True

    def step10_full_gas(self):
        """Step 10: in correct position, full gas"""
        self.get_logger().info("Step 10: Full Gas") 
        self.move_forward(duration=10, speed=127)

def main(args=None):
    rclpy.init(args=args)
    
    planner = Planner()
    
    try:
        planner.get_logger().info("Starting planner test scenarios...")

        # planner.move_forward(duration=1.0, speed=20)  # Move up to start
        # planner.move_backward(duration=1.0, speed=20)
        # planner.move_left(duration=1.0, speed=20)
        # planner.move_right(duration=1.0, speed=20)
        # planner.stop_all_motors()
        # time.sleep(5.0)
        # planner.rotate_left(duration=2.0, speed=20)
        #wait a bit
        time_start = time.time()
        while time.time() < time_start + 5:
            print("Waiting for 5 seconds before next action...")

        # planner.rotate_right(duration=1.0, speed=20)
        planner.set_depth(depth=2)
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