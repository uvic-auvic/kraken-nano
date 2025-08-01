import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64
sys.path.append("/home/ubuntu/Documents/uvic/kraken-nano/ROS/ws/src/kraken/kraken/include")
from simulation import Simulation
from std_msgs.msg import String
from pid import PID
from motor_board import MotorBoard, System
from custom.msg import PoseE

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # # self.motor_board = MotorBoard('/dev/null', baudrate=115200, timeout=0.1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sim = Simulation(self)
        self.sim.left(10)

        # PID controllers for each movement type
        self.forward_pid = PID(1.0, 0.0, 0.0)
        self.right_pid = PID(1.0, 0.0, 0.0)
        self.up_pid = PID(1.0, 0.0, 0.0)
        self.yaw_pid = PID(1.0, 0.0, 0.0)
        self.roll_pid = PID(1.0, 0.0, 0.0)
        self.pitch_pid = PID(1.0, 0.0, 0.0)

        # Store last measurements for each axis
        self.current_forward = 0.0
        self.current_right = 0.0
        self.current_up = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0

        self.target_forward = 0.0
        self.target_right = 0.0
        self.target_up = 0.0
        self.target_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0

        self.status_publisher = self.create_publisher(
            String, '/controller/status', 10)
        self.pose_publisher = self.create_publisher(
            String, '/controller/pose', 10)
        self.error_publisher = self.create_publisher(
            String, '/controller/error', 10)
        self.warning_publisher = self.create_publisher(
            String, '/controller/warning', 10)

        self.planner_task_subscription = self.create_subscription(
            String,
            '/planner/task',
            self.planner_task_callback,
            10)
        self.state_estimator_pose_subscription = self.create_subscription(
            PoseE,
            '/state_estimator/pose',
            self.state_estimator_pose_callback,
            10)

    def timer_callback(self):
        # Example: update up_pid (depth) using altimeter
        altimeter = self.sim.altimeter
        if altimeter is not None:
            self.current_up = altimeter.vertical_position
            # Assume setpoint is set elsewhere, dt is timer_period
            up_output = self.up_pid.update(self.current_up, 0.5)
            self.get_logger().info(f"[UP] Depth: {self.current_up:.2f}, PID output: {up_output:.2f}")
            # Here you would send up_output to the motor board


    def planner_task_callback(self, msg):
        # This method will be called when a new task is received from the planner
        self.get_logger().info(f'Received task: {msg.data}')
        self.recieve_task(msg.data)

    def state_estimator_pose_callback(self, msg):
        # This method will be called when a new pose is received from the state estimator
        self.get_logger().info(f'Received pose: {msg.data}')
        self.receive_pose(msg.data)

    # TODO: Implement a task receiving method
    def recieve_task(self, task):
        self.get_logger().info(f'Received task: {task}')
        # Expecting task as a dict-like object (from JSON or similar)
        if isinstance(task, str):
            import json
            try:
                task = json.loads(task)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse task as JSON: {e}")
                return

        task_type = task.get("type")
        if task_type == "move":
            direction = task.get("direction")
            magnitude = float(task.get("magnitude", 0))
            # Compute new setpoint based on last known pose
            if not hasattr(self, 'last_pose'):
                self.get_logger().warn("No pose received yet; cannot move.")
                return
            if direction == "x":
                target_x = self.last_pose.point.x + magnitude
                self.forward(target_x)
            elif direction == "y":
                target_y = self.last_pose.point.y + magnitude
                self.right(target_y)
            elif direction == "z":
                target_z = self.last_pose.point.z + magnitude
                self.up(target_z)
            elif direction == "yaw":
                target_yaw = self.last_pose.rot.yaw + magnitude
                self.yaw(target_yaw)
            elif direction == "roll":
                target_roll = self.last_pose.rot.roll + magnitude
                self.roll(target_roll)
            elif direction == "pitch":
                target_pitch = self.last_pose.rot.pitch + magnitude
                self.pitch(target_pitch)
            else:
                self.get_logger().warn(f"Unknown move direction: {direction}")
        elif task_type == "dropper":
            state = task.get("state")
            if state in ("on", "off"):
                self.set_dropper(state)
            else:
                self.get_logger().warn(f"Unknown dropper state: {state}")
        elif task_type == "grabber":
            state = task.get("state")
            if state in ("on", "off"):
                self.set_grabber(state)
            else:
                self.get_logger().warn(f"Unknown grabber state: {state}")
        elif task_type == "torpedo":
            state = task.get("state")
            if state in ("on", "off"):
                self.set_torpedo(state)
            else:
                self.get_logger().warn(f"Unknown torpedo state: {state}")
        else:
            self.get_logger().warn(f"Unknown task type: {task_type}")

    # TODO: Implement a pose receiving method    
    def receive_pose(self, pose):
        self.get_logger().info(f'Received pose: {pose}')
        # Expecting pose as a JSON string with 'point' (3D) and 'rot' (Euler [yaw, pitch, roll])
        import json
        try:
            self.last_pose = pose

            self.current_forward = pose.point.x
            self.current_right = pose.point.y
            self.current_up = pose.point.z
            self.current_yaw = pose.rot.yaw
            self.current_pitch = pose.rot.pitch
            self.current_roll = pose.rot.roll
        except Exception as e:
            self.get_logger().warn(f"Failed to parse pose: {e}")

        # Call all positional movement methods with their current setpoints
        self.forward(self.forward_pid.setpoint)
        self.right(self.right_pid.setpoint)
        self.up(self.up_pid.setpoint)
        self.yaw(self.yaw_pid.setpoint)
        self.roll(self.roll_pid.setpoint)
        self.pitch(self.pitch_pid.setpoint)

    # TODO: Implement a method to receive a task result
    def forward(self):
        setpoint = self.target_forward
        self.get_logger().info(f'Forward setpoint: {setpoint}')
        self.forward_pid.setpoint = setpoint
        output = self.forward_pid.update(self.current_forward, 0.5)
        self.get_logger().info(f"[FORWARD] Measured: {self.current_forward:.2f}, PID output: {output:.2f}")
        # self.motor_board.forward(int(max(min(output, 127), -128)))
        self.sim.forward(output)

    # TODO: Implement a method to set right movement speed
    def right(self):
        setpoint = self.target_right
        self.get_logger().info(f'Right setpoint: {setpoint}')
        self.right_pid.setpoint = setpoint
        output = self.right_pid.update(self.current_right, 0.5)
        self.get_logger().info(f"[RIGHT] Measured: {self.current_right:.2f}, PID output: {output:.2f}")
        # self.motor_board.right(int(max(min(output, 127), -128)))
        self.sim.right(output)

    # TODO: Implement a method to set up movement speed
    def up(self):
        setpoint = self.target_up
        self.get_logger().info(f'Up setpoint: {setpoint}')
        self.up_pid.setpoint = setpoint
        output = self.up_pid.update(self.current_up, 0.5)
        self.get_logger().info(f"[UP] Measured: {self.current_up:.2f}, PID output: {output:.2f}")
        # self.motor_board.vertical(int(max(min(output, 127), -128)))
        self.sim.up(output)

    # TODO: Implement a method to set yaw speed
    def yaw(self):
        setpoint = self.target_yaw
        self.get_logger().info(f'Yaw setpoint: {setpoint}')
        self.yaw_pid.setpoint = setpoint
        output = self.yaw_pid.update(self.current_yaw, 0.5)
        self.get_logger().info(f"[YAW] Measured: {self.current_yaw:.2f}, PID output: {output:.2f}")
        # self.motor_board.yaw(int(max(min(output, 127), -128)))

    # TODO: Implement a method to set roll speed
    def roll(self):
        setpoint = self.target_roll
        self.get_logger().info(f'Roll setpoint: {setpoint}')
        self.roll_pid.setpoint = setpoint
        output = self.roll_pid.update(self.current_roll, 0.5)
        self.get_logger().info(f"[ROLL] Measured: {self.current_roll:.2f}, PID output: {output:.2f}")
        # self.motor_board.roll(int(max(min(output, 127), -128)))

    # TODO: Implement a method to set pitch speed
    def pitch(self):
        setpoint = self.target_pitch
        self.get_logger().info(f'Pitch setpoint: {setpoint}')
        self.pitch_pid.setpoint = setpoint
        output = self.pitch_pid.update(self.current_pitch, 0.5)
        self.get_logger().info(f"[PITCH] Measured: {self.current_pitch:.2f}, PID output: {output:.2f}")
        # self.motor_board.pitch(int(max(min(output, 127), -128)))

    # TODO: Implement a method to set the dropper
    def set_dropper(self, ball):
        self.get_logger().info(f'Dropper set to: {ball}')
    
    # TODO: Implement a method to set the torpedo
    def set_torpedo(self, torpedo):
        self.get_logger().info(f'Torpedo set to: {torpedo}')
    
    # TODO: Implement a method to set the grabber
    def set_grabber(self, state):
        self.get_logger().info(f'Grabber state set to: {state}')

    # TODO: Implement a method to set task result
    def task_result(self, result):
        self.get_logger().info(f'Task result: {result}')

    # TODO: Implement a method to set the status of the controller
    def status_callback(self):
        # This method can be used to periodically check the status of the controller
        self.get_logger().info('Controller is running...')

        # You can add more logic here to check the status of various components
    
    # TODO: Implement a method to set the pose of the controller
    def set_pose_callback(self, pose):
        # This method can be used to set the pose of the controller
        self.get_logger().info(f'Setting pose: {pose}')

        # You can add more logic here to handle the pose setting

        # For example, you might want to publish the pose to a topic or update an internal state

    # TODO: Implement methods to handle errors and warnings
    def error_callback(self, error_message):
        # This method can be used to handle errors in the controller
        self.get_logger().error(f'Error occurred: {error_message}')

        # You can add more logic here to handle the error, such as retrying an operation or logging it

        # For example, you might want to publish the error message to a topic or take corrective actions    

    # TODO: Implement a method to handle warnings
    def warning_callback(self, warning_message):
        # This method can be used to handle warnings in the controller
        self.get_logger().warn(f'Warning: {warning_message}')

        # You can add more logic here to handle the warning, such as logging it or notifying other components

        # For example, you might want to publish the warning message to a topic or take preventive actions
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sim = Simulation(self)
        self.sim.left(10)

    def timer_callback(self):
        self.get_logger().info('Controller Running')
        altimeter = self.sim.altimeter
        if altimeter is not None:
            self.get_logger().info(str(altimeter.vertical_position))


def main(args=None):
    rclpy.init(args=args)

    controller = Controller() 

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
