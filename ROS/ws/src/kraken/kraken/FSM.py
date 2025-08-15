import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import json
import time
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    STEP_01_INITIAL_DESCENT = "step_01_initial_descent"
    STEP_02_YAW_SEARCH = "step_02_yaw_search"
    STEP_03_GATE_DETECTION = "step_03_gate_detection"
    STEP_04_GATE_APPROACH = "step_04_gate_approach"
    STEP_05_GATE_TRANSIT = "step_05_gate_transit"
    STEP_06_POST_GATE_SEARCH = "step_06_post_gate_search"
    EMERGENCY = "emergency"

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        
        # Initialize state
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        self.step_start_time = None
        self.step_timeout = 30.0  # Default step timeout in seconds
        
        # Publishers
        self.state_publisher = self.create_publisher(String, '/fsm/current_state', 10)
        self.command_publisher = self.create_publisher(String, '/fsm/command', 10)
        self.planner_command_publisher = self.create_publisher(String, '/planner/manual_command', 10)
        
        # Subscribers
        self.objects_subscriber = self.create_subscription(
            Int32MultiArray, 'objects', self.objects_callback, 10)
        self.object_positions_subscriber = self.create_subscription(
            String, 'object_positions', self.object_positions_callback, 10)
        self.task_subscriber = self.create_subscription(
            String, '/planner/task', self.task_callback, 10)
        self.search_space_subscriber = self.create_subscription(
            String, '/planner/search_space', self.search_space_callback, 10)
        
        # IMU data subscriber (to be implemented later)
        self.imu_subscriber = self.create_subscription(
            Float32MultiArray, '/imu/data', self.imu_callback, 10)
        
        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.state_machine_update)  # 10Hz
        
        # State variables
        self.detected_objects = [0] * 8  # ['Sawfish Gate Banner', 'Shark Gate Banner', 'Full Gate', 'Red Slalom', 'White Slalom', 'Full Torpedo Banner', 'Sawfish Torpedo Hole', 'Shark Torpedo Hole']
        self.object_positions = {}  # Store positioning data for detected objects
        self.current_task = None
        self.search_area = None
        
        # IMU data (to be populated by IMU callback)
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'depth': 0.0,
            'linear_accel': [0.0, 0.0, 0.0],
            'angular_vel': [0.0, 0.0, 0.0]
        }
        
        # Mission tracking variables
        self.gate_passed = False
        self.target_depth = 2.0  # meters
        self.mission_start_time = None
        
        self.get_logger().info('Simplified FSM Node initialized with 7 basic navigation steps')

    def objects_callback(self, msg):
        """Callback for object detection updates"""
        self.detected_objects = msg.data
        self.get_logger().debug(f'Received objects: {self.detected_objects}')

    def object_positions_callback(self, msg):
        """Callback for object positioning data"""
        try:
            self.object_positions = json.loads(msg.data)
            self.get_logger().debug(f'Received object positions: {len(self.object_positions)} objects')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing object positions: {str(e)}')

    def get_gate_alignment_data(self):
        """Get gate positioning data for alignment control"""
        gate_data = {}
        
        if 'Full Gate' in self.object_positions:
            gate_data = self.object_positions['Full Gate']
            
        return {
            'detected': 'Full Gate' in self.object_positions,
            'relative_x': gate_data.get('relative_x', 0.0),  # -1 (left) to 1 (right)
            'relative_y': gate_data.get('relative_y', 0.0),  # -1 (up) to 1 (down) 
            'quadrant': gate_data.get('quadrant', 0),
            'distance': gate_data.get('distance', 0.0),
            'confidence': gate_data.get('confidence', 0.0)
        }

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

    def imu_callback(self, msg):
        """Callback for IMU data updates - TO BE IMPLEMENTED"""
        try:
            # Expected IMU data format: [roll, pitch, yaw, depth, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            if len(msg.data) >= 10:
                self.imu_data = {
                    'roll': msg.data[0],
                    'pitch': msg.data[1],
                    'yaw': msg.data[2],
                    'depth': msg.data[3],
                    'linear_accel': [msg.data[4], msg.data[5], msg.data[6]],
                    'angular_vel': [msg.data[7], msg.data[8], msg.data[9]]
                }
                self.get_logger().debug(f'IMU Update - Depth: {self.imu_data["depth"]:.2f}m, Yaw: {self.imu_data["yaw"]:.1f}°')
        except Exception as e:
            self.get_logger().error(f'Error parsing IMU data: {str(e)}')

    def transition_to_state(self, new_state: RobotState):
        """Transition to a new state"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.step_start_time = time.time()
            
            # Publish state change
            state_msg = String()
            state_msg.data = self.current_state.value
            self.state_publisher.publish(state_msg)
            
            self.get_logger().info(f'State transition: {self.previous_state.value} -> {self.current_state.value}')

    def state_machine_update(self):
        """Main state machine logic - called at 10Hz"""
        
        # Check for step timeout
        if self.step_start_time and time.time() - self.step_start_time > self.step_timeout:
            self.get_logger().warn(f'Step timeout in state: {self.current_state.value}')
            self.handle_step_timeout()
            return
        
        # Execute current step
        if self.current_state == RobotState.IDLE:
            self.step_idle()
        elif self.current_state == RobotState.STEP_01_INITIAL_DESCENT:
            self.step_01_initial_descent()
        elif self.current_state == RobotState.STEP_02_YAW_SEARCH:
            self.step_02_yaw_search()
        elif self.current_state == RobotState.STEP_03_GATE_DETECTION:
            self.step_03_gate_detection()
        elif self.current_state == RobotState.STEP_04_GATE_APPROACH:
            self.step_04_gate_approach()
        elif self.current_state == RobotState.STEP_05_GATE_TRANSIT:
            self.step_05_gate_transit()
        elif self.current_state == RobotState.STEP_06_POST_GATE_SEARCH:
            self.step_06_post_gate_search()

    # ================== STEP FUNCTIONS ==================

    def step_idle(self):
        """IDLE: Wait for mission start command"""
        # TODO: Add your logic for when to start mission
        # For now, auto-start after 2 seconds
        if not self.mission_start_time:
            self.mission_start_time = time.time()
        elif time.time() - self.mission_start_time > 2.0:
            self.transition_to_state(RobotState.STEP_01_INITIAL_DESCENT)

    def step_01_initial_descent(self):
        """STEP 1: Initial descent to competition depth"""

        if not hasattr(self, '_descent_command_sent') or not self._descent_command_sent:
            self.send_planner_command("move_down", {"speed": 127, "duration": 3.0})
            self._descent_command_sent = True
            self.get_logger().info("Starting initial descent at speed 127 for 3 seconds")
        
        # Transition conditions (in order of priority):
        # 1. Reached target depth early
        if self.imu_data['depth'] >= self.target_depth:
            self.get_logger().info(f"Reached target depth early: {self.imu_data['depth']:.2f}m")
            self._descent_command_sent = False  # Reset for potential re-entry
            self.transition_to_state(RobotState.STEP_02_YAW_SEARCH)
        
        # 2. Time-based transition (2-3 seconds as you specified)
        elif self.step_start_time and time.time() - self.step_start_time >= 2.5:  # 2.5 seconds
            self.get_logger().info(f"Descent time complete. Current depth: {self.imu_data['depth']:.2f}m")
            self._descent_command_sent = False  # Reset for potential re-entry
            self.transition_to_state(RobotState.STEP_02_YAW_SEARCH)

    def step_02_yaw_search(self):
        """STEP 3: Clockwise yaw search for gate detection"""
        # Perform continuous clockwise yaw search until gate + banner detected
        # - Rotate clockwise indefinitely 
        # - Search for gate AND (sawfish OR shark) banner simultaneously
        # - Maintain depth and position during search
        
        # Send continuous clockwise yaw command
        self.send_planner_command("yaw_search_clockwise", {
            "yaw_speed": 30, 
            "maintain_depth": self.target_depth,
            "maintain_position": True
        })
        
        # Transition condition: detect gate AND either sawfish or shark banner
        gate_detected = self.detected_objects[2]  # Full Gate
        sawfish_banner = self.detected_objects[0]  # Sawfish Gate Banner  
        shark_banner = self.detected_objects[1]  # Shark Gate Banner
        
        if gate_detected and (sawfish_banner or shark_banner):
            banner_type = "sawfish" if sawfish_banner else "shark"
            self.get_logger().info(f"Gate and {banner_type} banner detected - proceeding to gate transit")
            self.transition_to_state(RobotState.STEP_03_GATE_DETECTION)

    def step_03_gate_detection(self):
        """STEP 4: Gate transit - move through gate while maintaining proper alignment"""
        # Navigate through the detected gate
        # - Move forward through gate center
        # - Ensure left side of gate stays on left of camera frame
        # - Ensure right side of gate stays on right of camera frame
        # - Maintain proper depth and orientation during transit
        
        gate_detected = self.detected_objects[2]  # Full Gate
        sawfish_banner = self.detected_objects[0]  # Sawfish Gate Banner  
        shark_banner = self.detected_objects[1]  # Shark Gate Banner
        
        if gate_detected and (sawfish_banner or shark_banner):
            # Get gate alignment data
            gate_alignment = self.get_gate_alignment_data()
            banner_type = "sawfish" if sawfish_banner else "shark"
            
            # Continue moving forward through gate with alignment control
            self.send_planner_command("gate_transit", {
                "speed": 127,
                "maintain_alignment": True,
                "keep_gate_centered": True,
                "gate_type": banner_type,
                "maintain_depth": self.target_depth,
                "gate_relative_x": gate_alignment['relative_x'],
                "gate_relative_y": gate_alignment['relative_y'],
                "gate_quadrant": gate_alignment['quadrant'],
                "gate_distance": gate_alignment['distance']
            })
            
            self.get_logger().debug(f"Gate alignment - X: {gate_alignment['relative_x']:.2f}, Y: {gate_alignment['relative_y']:.2f}, Q: {gate_alignment['quadrant']}, Dist: {gate_alignment['distance']:.2f}m")
            
            # Transition condition: gate no longer visible (passed through) or timeout
            if self.step_start_time and time.time() - self.step_start_time > 8.0:
                self.gate_passed = True
                self.get_logger().info("Gate transit completed - moving to post-gate search")
                self.transition_to_state(RobotState.STEP_06_POST_GATE_SEARCH)
        else:
            # Lost sight of gate or banner, go back to yaw search
            self.get_logger().warn("Lost gate or banner detection - returning to yaw search")
            self.transition_to_state(RobotState.STEP_02_YAW_SEARCH)

    def step_04_gate_approach(self):
        """STEP 4: Approach the detected gate"""
        # TODO: Implement gate approach logic
        # - Center the gate in camera view
        # - Approach at controlled speed
        # - Maintain proper depth and orientation
        
        if self.detected_objects[2]:  # Still see the gate
            self.send_planner_command("approach_target", {"target": "gate", "speed": 40})
            # Transition condition: close enough to gate (use depth sensor or time)
            if self.step_start_time and time.time() - self.step_start_time > 5.0:
                self.transition_to_state(RobotState.STEP_05_GATE_TRANSIT)
        else:
            # Lost the gate, go back to detection
            self.transition_to_state(RobotState.STEP_03_GATE_DETECTION)

    def step_05_gate_transit(self):
        """STEP 5: Pass through the gate"""
        # TODO: Implement gate transit logic
        # - Pass through gate center
        # - Monitor for successful passage
        # - Maintain course and speed
        
        self.send_planner_command("move_forward", {"speed": 60, "duration": 3.0})
        
        # Transition condition: passed through gate (no longer detected behind)
        if self.step_start_time and time.time() - self.step_start_time > 3.0:
            self.gate_passed = True
            self.transition_to_state(RobotState.STEP_06_POST_GATE_SEARCH)

    def step_06_post_gate_search(self):
        """STEP 6: Search for next task after gate - Final step"""
        # TODO: Implement post-gate search
        # - Search for torpedo targets or other tasks
        # - Scan area methodically
        # - Maintain search depth
        
        self.send_planner_command("search_pattern", {"pattern": "post_gate", "duration": 2.0})
        
        # Final step - mission simplified to basic navigation
        # Could transition back to IDLE or continue searching
        if self.step_start_time and time.time() - self.step_start_time > 10.0:
            self.get_logger().info("Mission complete - simplified FSM finished at step 7")
            self.transition_to_state(RobotState.IDLE)

    # ================== HELPER FUNCTIONS ==================

    def handle_step_timeout(self):
        """Handle step timeout situations"""
        self.get_logger().warn(f"Timeout in step: {self.current_state.value}")
        # Could implement recovery logic or advance to next step
        # For now, just continue to next logical step
        

        if self.current_state == RobotState.STEP_01_INITIAL_DESCENT:
            self.transition_to_state(RobotState.STEP_02_YAW_SEARCH)
        elif self.current_state == RobotState.STEP_02_YAW_SEARCH:
            self.transition_to_state(RobotState.STEP_06_POST_GATE_SEARCH)  # Skip gate if not found
        # Add more timeout recovery logic as needed

    def send_planner_command(self, command_type: str, parameters: dict = None):
        """Send command to planner for execution"""
        command = {
            'command_type': command_type,
            'state': self.current_state.value,
            'parameters': parameters or {},
            'timestamp': str(self.get_clock().now().nanoseconds)
        }
        
        command_msg = String()
        command_msg.data = json.dumps(command)
        self.planner_command_publisher.publish(command_msg)
        
        self.get_logger().debug(f'Sent planner command: {command_type}')

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