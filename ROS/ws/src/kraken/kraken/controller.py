import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64

sys.path.append("/home/vboxuser/kraken-nano/ROS/ws/src/kraken/kraken/include")

from simulation import Simulation
from pid import PID

from custom.msg import PoseE

class Controller(Node):

        def __init__(self):
                super().__init__('controller')
                
                self.subscription = self.create_subscription(PoseE, '/state_estimator/pose', self.pose_callback, 10)
                pid_period = 0.01  # seconds
                self.pid_timer = self.create_timer(pid_period, self.pid_callback)
                self.logger = self.get_logger()
                
                self.sim = Simulation(self)
                
                self.pose = None
                self.forward_pid = PID(0, 0, 4, 0, 8)
                self.up_pid = PID(0, 0, 4, 0, 8)
                self.left_pid = PID(0, 0, 4, 0, 8)
                self.yaw_pid = PID(0, 1, 5, 0, 20)

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
                        
                        self.logger.info(str(self.pose.rot))
                       
                
	        
	        
        def pose_callback(self, msg):
                self.pose = msg


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
