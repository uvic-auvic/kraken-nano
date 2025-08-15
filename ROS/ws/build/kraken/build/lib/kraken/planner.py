import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from enum import Enum

class TaskType(Enum):
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    SET_DEPTH = "setdepth"
    STOP_MOTORS = "stop_motors"

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

    def execute_task_step(self):
        """Execute one step of the current task"""
        task_type = self.current_task['task_type']
        parameters = self.current_task['parameters']
        
        # Check if task should timeout
        if self.task_start_time:
            elapsed_time = time.time() - self.task_start_time
            max_duration = parameters.get('duration', 10.0)  # Default 10 seconds
            
            if elapsed_time > max_duration:
                self.get_logger().info(f'Task {task_type} timed out')
                self.complete_current_task()
                return
        
        # Execute task based on type
        if task_type == TaskType.MOVE_FORWARD.value:
            self.send_motor_command("forward", parameters.get('speed', 60))
            
        elif task_type == TaskType.MOVE_BACKWARD.value:
            self.send_motor_command("backward", parameters.get('speed', 60))
            
        elif task_type == TaskType.MOVE_LEFT.value:
            self.send_motor_command("left", parameters.get('speed', 60))
            
        elif task_type == TaskType.MOVE_RIGHT.value:
            self.send_motor_command("right", parameters.get('speed', 60))
            
        elif task_type == TaskType.MOVE_UP.value:
            self.send_motor_command("up", parameters.get('speed', 60))
            
        elif task_type == TaskType.MOVE_DOWN.value:
            self.send_motor_command("down", parameters.get('speed', 60))
            
        elif task_type == TaskType.ROTATE_LEFT.value:
            self.send_motor_command("yaw_ccw", parameters.get('speed', 60))
            
        elif task_type == TaskType.ROTATE_RIGHT.value:
            self.send_motor_command("yaw_cw", parameters.get('speed', 60))
            
        elif task_type == TaskType.STOP_MOTORS.value:
            self.send_motor_command("stop", 0)
            self.complete_current_task()

    def complete_current_task(self):
        """Mark current task as complete"""
        if self.current_task:
            self.get_logger().info(f'Completed task: {self.current_task["task_type"]}')
            self.send_motor_command("stop", 0)  # Stop motors between tasks
            self.current_task = None
            self.task_start_time = None

    def send_motor_command(self, command: str, speed: int):
        """Send motor command to controller"""
        motor_msg = {
            'command': command,
            'speed': speed,
            'timestamp': str(self.get_clock().now().nanoseconds)
        }
        
        message = String()
        message.data = json.dumps(motor_msg)
        self.motor_command_publisher.publish(message)

    # Convenience methods for basic movements
    def move_forward(self, duration: float = 3.0, speed: int = 60):
        """Add move forward task"""
        self.add_task(TaskType.MOVE_FORWARD, {"duration": duration, "speed": speed})

    def move_backward(self, duration: float = 3.0, speed: int = 60):
        """Add move backward task"""
        self.add_task(TaskType.MOVE_BACKWARD, {"duration": duration, "speed": speed})

    def move_left(self, duration: float = 2.0, speed: int = 50):
        """Add move left task"""
        self.add_task(TaskType.MOVE_LEFT, {"duration": duration, "speed": speed})

    def move_right(self, duration: float = 2.0, speed: int = 50):
        """Add move right task"""
        self.add_task(TaskType.MOVE_RIGHT, {"duration": duration, "speed": speed})

    def move_up(self, duration: float = 2.0, speed: int = 50):
        """Add move up task"""
        self.add_task(TaskType.MOVE_UP, {"duration": duration, "speed": speed})

    def move_down(self, duration: float = 2.0, speed: int = 50):
        """Add move down task"""
        self.add_task(TaskType.MOVE_DOWN, {"duration": duration, "speed": speed})

    def rotate_left(self, duration: float = 2.0, speed: int = 40):
        """Add rotate left task"""
        self.add_task(TaskType.ROTATE_LEFT, {"duration": duration, "speed": speed})

    def rotate_right(self, duration: float = 2.0, speed: int = 40):
        """Add rotate right task"""
        self.add_task(TaskType.ROTATE_RIGHT, {"duration": duration, "speed": speed})

    def stop_all_motors(self):
        """Immediately stop all motors"""
        self.add_task(TaskType.STOP_MOTORS, {"duration": 0.1})

    def set_depth(self, depth: float):
        self.add_task(TaskType.SET_DEPTH, {"speed": depth})

def main(args=None):
    rclpy.init(args=args)
    
    planner = Planner()
    
    try:
        planner.get_logger().info("Starting planner test scenarios...")
        
        # ========================================
        # Step 1 go to depth
        # ========================================
        planner.get_logger().info("Step 1: Go to Depth")
        planner.set_depth(speed=1)
        time.sleep(0.1)
        planner.get_logger().info("Step 1: Completed")

        
        # ========================================
        # Step 2: yaw rotation 
        # ========================================
        isGateFound = False
        while not isGateFound:
            if self.objects[2] == 1:
                isGateFound = True
                break
            planner.get_logger().info("Step 2: Yaw Rotation")
            planner.rotate_left(duration=1.0, speed=100)
            time.sleep(0.1)
        
        # ========================================
        # Step 3: Yaw slower till gate centered
        # ========================================
        isGateCentered = False
        while not isGateCentered:
            if self.detection_info.get('center_x'):
                if self.detection_info.get('class_name') == 'Full Gate':
                    center_x = self.detection_info['center_x']
                    if center_x > 400:
                        planner.rotate_right(duration=0.5, speed=50)
                    elif center_x < 240:
                        planner.rotate_left(duration=0.5, speed=50)

            if center_x > 240 and center_x < 400:
                planner.get_logger().info("Gate is centered")
                isGateCentered = True

        # ========================================
        # Step 4: gate centered go go go
        # ========================================
        isGateFound =  True
        while isGateFound:
            if self.objects[2] == 0:
                isGateFound = False
            planner.move_forward(duration=0.5, speed=127)  # Move through gate
        # ========================================
        # Step 5: gate centered go go go
        # ========================================
        planner.move_forward(duration=2, speed=127)  # Move through gate

        # ========================================
        # Step 6: find the slalom 
        # ========================================
        isSlalomFound = False
        gatePayload = None
        while not isSlalomFound:
            if self.objects[3] == 1:
                isSlalomFound = True
            planner.get_logger().info("Step 6: Searching for Slalom")
            planner.rotate_left(duration=0.5, speed=100)
        # ========================================
        # Step 7: is Slalom Red Centered
        # ========================================
        isSlalomRedCentered = False
        while not isSlalomRedCentered:
            if self.detection_info.get('center_x'):
                if self.detection_info.get('class_name') == 'Red Slalom':
                    center_x = self.detection_info['center_x']
                    if center_x > 400:
                        planner.rotate_right(duration=0.5, speed=50)
                    elif center_x < 240:
                        planner.rotate_left(duration=0.5, speed=50)

            if center_x > 240 and center_x < 400:
                planner.get_logger().info("Gate is centered")
                isGateCentered = True
        # ========================================
        # Step 8: get slalom Red distance to 2 meters away
        # ========================================
        isSlalomRedTwoMetersAway = False
        while not isSlalomRedTwoMetersAway:
            if self.detection_info.get('distance'):
                if self.detection_info.get('class_name') == 'Red Slalom':
                    distance = self.detection_info['distance']
                    if distance > 2.0:
                        planner.move_forward(duration=0.5, speed=50)
                    else:
                        planner.get_logger().info("Slalom Red is 2 meters away")
                        isSlalomRedTwoMetersAway = True
        # ========================================
        # Step 9: get slalom Red into the far right third of image 
        # ========================================
        isInRightThird = False
        while not isInRightThird:
            if self.detection_info.get('center_x'):
                center_x = self.detection_info['center_x']
                if self.detection_info.get('class_name') == 'Red Slalom':
                    planner.move_right(duration=0.5, speed=50)
            if center_x > 480:
                planner.get_logger().info("Gate is centered")
                isInRightThird = True
        
        # ========================================
        # Step 10: in correct position, full gas 
        # ========================================
        planner.get_logger().info("Step 10: Full Gas") 
        planner.move_forward(duration=10, speed=127)
        
    except KeyboardInterrupt:
        planner.get_logger().info('Planner shutting down...')
        planner.stop_all_motors()  # Stop motors on shutdown
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()