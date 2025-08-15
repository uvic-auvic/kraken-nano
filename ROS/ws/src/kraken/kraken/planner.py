import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
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
    SEARCH_PATTERN = "search_pattern"
    APPROACH_TARGET = "approach_target"
    STOP_MOTORS = "stop_motors"

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        
        # Create publishers for the planner topics
        self.task_publisher = self.create_publisher(String, '/planner/task', 10)
        self.search_space_publisher = self.create_publisher(String, '/planner/search_space', 10)
        self.motor_command_publisher = self.create_publisher(String, '/motor/command', 10)
        
        # Subscribe to object detections
        self.objects_subscriber = self.create_subscription(
            Int32MultiArray, 'objects', self.objects_callback, 10)
        
        # Task management
        self.current_task = None
        self.task_queue = []
        self.detected_objects = [0] * 8
        self.task_start_time = None
        
        # Timer for task execution
        self.task_timer = self.create_timer(0.5, self.execute_current_task)  # 2Hz
        
        self.get_logger().info('Planner Node initialized')

    def objects_callback(self, msg):
        """Callback for object detection updates"""
        self.detected_objects = msg.data
        self.get_logger().debug(f'Received objects: {self.detected_objects}')
        
        # React to object detections
        self.react_to_detections()

    def react_to_detections(self):
        """React to detected objects by planning appropriate tasks"""
        # Check for Full Gate (index 2)
        if self.detected_objects[2] == 1 and not self.is_task_type_active(TaskType.APPROACH_TARGET):
            self.add_task(TaskType.APPROACH_TARGET, {"target": "gate", "duration": 5.0})
            
        # Check for Red/White Slalom (indices 3, 4)
        elif (self.detected_objects[3] == 1 or self.detected_objects[4] == 1) and not self.is_task_type_active(TaskType.SEARCH_PATTERN):
            self.add_task(TaskType.SEARCH_PATTERN, {"pattern": "slalom", "duration": 8.0})
            
        # Check for Torpedo targets (indices 6, 7)
        elif (self.detected_objects[6] == 1 or self.detected_objects[7] == 1) and not self.is_task_type_active(TaskType.APPROACH_TARGET):
            self.add_task(TaskType.APPROACH_TARGET, {"target": "torpedo", "duration": 3.0})

    def is_task_type_active(self, task_type):
        """Check if a task of given type is currently active or queued"""
        if self.current_task and self.current_task['task_type'] == task_type.value:
            return True
        return any(task['task_type'] == task_type.value for task in self.task_queue)

    def add_task(self, task_type: TaskType, parameters: dict = None):
        """Add a task to the queue"""
        task = {
            'task_type': task_type.value,
            'parameters': parameters or {},
            'priority': 1,
            'timestamp': self.get_clock().now().to_msg()
        }
        self.task_queue.append(task)
        self.get_logger().info(f'Added task: {task_type.value}')

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
            
        elif task_type == TaskType.SEARCH_PATTERN.value:
            self.execute_search_pattern(parameters)
            
        elif task_type == TaskType.APPROACH_TARGET.value:
            self.execute_approach_target(parameters)
            
        elif task_type == TaskType.STOP_MOTORS.value:
            self.send_motor_command("stop", 0)
            self.complete_current_task()

    def execute_search_pattern(self, parameters):
        """Execute a search pattern"""
        pattern = parameters.get('pattern', 'default')
        elapsed_time = time.time() - self.task_start_time
        
        if pattern == 'slalom':
            # Simple slalom pattern: left, forward, right, forward
            cycle_time = 2.0  # 2 seconds per movement
            phase = int(elapsed_time // cycle_time) % 4
            
            if phase == 0:
                self.send_motor_command("left", 50)
            elif phase == 1:
                self.send_motor_command("forward", 60)
            elif phase == 2:
                self.send_motor_command("right", 50)
            else:
                self.send_motor_command("forward", 60)
        else:
            # Default search: forward with slight turns
            if int(elapsed_time) % 4 < 2:
                self.send_motor_command("forward", 50)
            else:
                self.send_motor_command("yaw_cw", 40)

    def execute_approach_target(self, parameters):
        """Approach a detected target"""
        target = parameters.get('target', 'unknown')
        
        # Simple approach: move forward towards target
        if any(self.detected_objects[i] == 1 for i in [2, 6, 7]):  # Gate or torpedo detected
            self.send_motor_command("forward", 70)
        else:
            # Target lost, search
            self.send_motor_command("yaw_cw", 30)

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
        
    def publish_task(self, task_msg: dict):
        """Publishes task message to /planner/task"""
        try:
            # Convert dict to JSON string
            message = String()
            message.data = json.dumps(task_msg)
            
            # Publish the message
            self.task_publisher.publish(message)
            
            self.get_logger().info(f'Published task: {task_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing task: {str(e)}')

    def publish_search_space(self, search_space_msg: dict):
        """Publishes search space message to /planner/search_space"""
        try:
            # Convert dict to JSON string
            message = String()
            message.data = json.dumps(search_space_msg)
            
            # Publish the message
            self.search_space_publisher.publish(message)
            
            self.get_logger().info(f'Published search space: {search_space_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing search space: {str(e)}')

    # Convenience methods for manual task control
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

    def start_search_pattern(self, pattern_type: str = "default", duration: float = 10.0):
        """Start a search pattern"""
        self.add_task(TaskType.SEARCH_PATTERN, {"pattern": pattern_type, "duration": duration})

    def publish_task_by_name(self, task_name: str, priority: int = 1, parameters: dict = None):
        """Convenience method to publish a task by name"""
        task_msg = {
            'task_name': task_name,
            'priority': priority,
            'timestamp': self.get_clock().now().to_msg(),
            'parameters': parameters or {}
        }
        self.publish_task(task_msg)

    def publish_search_area(self, area_name: str, coordinates: list, search_type: str = 'default'):
        """Convenience method to publish a search area"""
        search_space_msg = {
            'area_name': area_name,
            'coordinates': coordinates,
            'search_type': search_type,
            'timestamp': self.get_clock().now().to_msg()
        }
        self.publish_search_space(search_space_msg)
        """Publishes task message to /planner/task"""
        try:
            # Convert dict to JSON string
            message = String()
            message.data = json.dumps(task_msg)
            
            # Publish the message
            self.task_publisher.publish(message)
            
            self.get_logger().info(f'Published task: {task_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing task: {str(e)}')

    def publish_search_space(self, search_space_msg: dict):
        """Publishes search space message to /planner/search_space"""
        try:
            # Convert dict to JSON string
            message = String()
            message.data = json.dumps(search_space_msg)
            
            # Publish the message
            self.search_space_publisher.publish(message)
            
            self.get_logger().info(f'Published search space: {search_space_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing search space: {str(e)}')

    def publish_task_by_name(self, task_name: str, priority: int = 1, parameters: dict = None):
        """Convenience method to publish a task by name"""
        task_msg = {
            'task_name': task_name,
            'priority': priority,
            'timestamp': self.get_clock().now().to_msg(),
            'parameters': parameters or {}
        }
        self.publish_task(task_msg)

    def publish_search_area(self, area_name: str, coordinates: list, search_type: str = 'default'):
        """Convenience method to publish a search area"""
        search_space_msg = {
            'area_name': area_name,
            'coordinates': coordinates,
            'search_type': search_type,
            'timestamp': self.get_clock().now().to_msg()
        }
        self.publish_search_space(search_space_msg)


def main(args=None):
    rclpy.init(args=args)
    
    planner = Planner()
    
    try:
        planner.get_logger().info("Starting planner test scenarios...")
        
        # ========================================
        # TEST SCENARIO 2: DESCEND (DIVE DOWN)
        # ========================================
        planner.get_logger().info("TEST 2: Descend/Dive")
        planner.move_down(duration=2.0, speed=127)
        # ========================================
        # TEST SCENARIO 1: MOVE FORWARD
        # ========================================
        planner.get_logger().info("TEST 1: Move Forward")
        planner.move_forward(duration=10.0, speed=127)
        planner.move_down(duration=10.0, speed=40)
        time.sleep(0.1)
        
        # ========================================
        # TEST SCENARIO 2: DESCEND (DIVE DOWN)
        # ========================================
        planner.get_logger().info("TEST 2: Descend/Dive")
        planner.move_up(duration=2.0, speed=100)
        planner.get_logger().info("TEST 2: Descend/Dive")
        planner.move_down(duration=2.0, speed=100)
        # time.sleep(0.1)
        
        # ========================================
        # TEST SCENARIO 3: YAW ROTATION
        # ========================================
        planner.get_logger().info("TEST 3: Yaw Rotation")
        planner.rotate_right(duration=15.0, speed=127)  # Rotate clockwise
        planner.move_down(duration=10.0, speed=40)
        time.sleep(0.5)
        planner.rotate_left(duration=3.0, speed=100)   # Rotate counter-clockwise
        time.sleep(0.1)
        
        # ========================================
        # TEST SCENARIO 4: COMBINATION MOVEMENT
        # ========================================
        # planner.get_logger().info("TEST 4: Combination Movement")
        # planner.move_down(duration=2.0, speed=60)     # Dive
        # time.sleep(0.1)
        # planner.move_forward(duration=4.0, speed=70)  # Move forward
        # time.sleep(0.1)
        # planner.rotate_right(duration=2.0, speed=40)  # Turn right
        # time.sleep(0.1)
        # planner.move_up(duration=2.0, speed=50)       # Surface
        # time.sleep(0.1)
        
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info('Planner shutting down...')
        planner.stop_all_motors()  # Stop motors on shutdown
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
