import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import json
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    NAVIGATING = "navigating"
    TASK_EXECUTION = "task_execution"
    EMERGENCY = "emergency"

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        
        # Initialize state
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        
        # Publishers
        self.state_publisher = self.create_publisher(String, '/fsm/current_state', 10)
        self.command_publisher = self.create_publisher(String, '/fsm/command', 10)
        
        # Subscribers
        self.objects_subscriber = self.create_subscription(
            Int32MultiArray, 'objects', self.objects_callback, 10)
        self.task_subscriber = self.create_subscription(
            String, '/planner/task', self.task_callback, 10)
        self.search_space_subscriber = self.create_subscription(
            String, '/planner/search_space', self.search_space_callback, 10)
        
        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.state_machine_update)  # 10Hz
        
        # State variables
        self.detected_objects = [0] * 8
        self.current_task = None
        self.search_area = None
        
        self.get_logger().info('FSM Node initialized')

    def objects_callback(self, msg):
        """Callback for object detection updates"""
        self.detected_objects = msg.data
        self.get_logger().debug(f'Received objects: {self.detected_objects}')

    def task_callback(self, msg):
        """Callback for task updates from planner"""
        try:
            self.current_task = json.loads(msg.data)
            self.get_logger().info(f'Received task: {self.current_task}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing task message: {str(e)}')

    def search_space_callback(self, msg):
        """Callback for search space updates from planner"""
        try:
            self.search_area = json.loads(msg.data)
            self.get_logger().info(f'Received search area: {self.search_area}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing search space message: {str(e)}')

    def transition_to_state(self, new_state: RobotState):
        """Transition to a new state"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            
            # Publish state change
            state_msg = String()
            state_msg.data = self.current_state.value
            self.state_publisher.publish(state_msg)
            
            self.get_logger().info(f'State transition: {self.previous_state.value} -> {self.current_state.value}')

    def state_machine_update(self):
        """Main state machine logic - called at 10Hz"""
        
        # Check for objects detected
        objects_detected = any(obj > 0 for obj in self.detected_objects)
        
        # State machine logic
        if self.current_state == RobotState.IDLE:
            if self.current_task:
                if objects_detected:
                    self.transition_to_state(RobotState.TASK_EXECUTION)
                else:
                    self.transition_to_state(RobotState.SEARCHING)
                    
        elif self.current_state == RobotState.SEARCHING:
            if objects_detected:
                self.transition_to_state(RobotState.NAVIGATING)
            elif not self.current_task:
                self.transition_to_state(RobotState.IDLE)
                
        elif self.current_state == RobotState.NAVIGATING:
            if not objects_detected:
                self.transition_to_state(RobotState.SEARCHING)
            elif self.is_close_to_target():
                self.transition_to_state(RobotState.TASK_EXECUTION)
                
        elif self.current_state == RobotState.TASK_EXECUTION:
            if self.is_task_complete():
                self.current_task = None
                self.transition_to_state(RobotState.IDLE)

    def is_close_to_target(self):
        """Check if robot is close enough to target to execute task"""
        # This would integrate with your depth/distance measurements
        # For now, just return True as placeholder
        return True

    def is_task_complete(self):
        """Check if current task is complete"""
        # Implement task completion logic here
        # For now, just return True as placeholder
        return True

    def publish_command(self, command_type: str, parameters: dict = None):
        """Publish a command based on current state"""
        command = {
            'command_type': command_type,
            'state': self.current_state.value,
            'parameters': parameters or {},
            'timestamp': self.get_clock().now().to_msg()
        }
        
        command_msg = String()
        command_msg.data = json.dumps(command)
        self.command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Published command: {command}')


def main(args=None):
    rclpy.init(args=args)
    
    fsm_node = FSMNode()
    
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        fsm_node.get_logger().info('FSM Node shutting down...')
    finally:
        fsm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()