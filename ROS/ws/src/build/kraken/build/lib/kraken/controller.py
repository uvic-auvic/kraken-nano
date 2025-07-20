import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64

sys.path.append("/home/ubuntu/Documents/uvic/kraken-nano/ROS/ws/src/kraken/kraken/include")

from simulation import Simulation
from std_msgs.msg import String


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sim = Simulation(self)
        self.sim.left(10)

        self.status_publisher = self.create_publisher(
            String, 'controller_status', 10)
        self.pose_publisher = self.create_publisher(
            String, 'controller_pose', 10)
        self.error_publisher = self.create_publisher(
            String, 'controller_error', 10)
        self.warning_publisher = self.create_publisher(
            String, 'controller_warning', 10)

        self.planner_task_subscription = self.create_subscription(
            String,
            'planner/task',
            self.planner_task_callback,
            10)
        self.state_estimator_pose_subscription = self.create_subscription(
            String,
            'state_estimator/pose',
            self.state_estimator_pose_callback,
            10)

    def timer_callback(self):
        #self.get_logger().info('Controller Running')
        altimeter = self.sim.get_altimeter()
        if altimeter is not None:
	        self.get_logger().info(str(altimeter.vertical_position))


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

    # TODO: Implement a pose receiving method    
    def receive_pose(self, pose):
        self.get_logger().info(f'Received pose: {pose}')

    # TODO: Implement a method to receive a task result
    def forward(self, speed):
        self.get_logger().info(f'Forward speed set to: {speed}')

    # TODO: Implement a method to set right movement speed
    def right(self, speed):
        self.get_logger().info(f'Right speed set to: {speed}')

    # TODO: Implement a method to set up movement speed
    def up(self, speed):
        self.get_logger().info(f'Up speed set to: {speed}')

    # TODO: Implement a method to set yaw speed
    def yaw(self, speed):
        self.get_logger().info(f'Yaw speed set to: {speed}')

    # TODO: Implement a method to set roll speed
    def roll(self, speed):
        self.get_logger().info(f'Roll speed set to: {speed}')

    # TODO: Implement a method to set pitch speed
    def pitch(self, speed):
        self.get_logger().info(f'Pitch speed set to: {speed}')

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
        altimeter = self.sim.get_altimeter()
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
