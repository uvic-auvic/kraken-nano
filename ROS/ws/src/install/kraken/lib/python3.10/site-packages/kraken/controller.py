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
                self.up_pid = PID(0, 0, 4, 0, 8)
                self.yaw_pid = PID(0, 1, 5, 0, 20)
                self.roll_pid = PID(0, 0, 4, 0, 8)
                self.pitch_pid = PID(0, 0, 4, 0, 8)

        def pid_callback(self):
                if self.pose:
                        up_speed = self.up_pid.calculate(self.pose.pos.z)
                        yaw_speed = self.yaw_pid.calculate(self.pose.rot.yaw)
                        #roll_speed = self.roll_pid.calculate(self.pose.rot.roll)
                        #pitch_speed = self.pitch_pid.calculate(self.pose.rot.pitch)
                        #self.sim.up(up_speed)
                        self.sim.yaw(yaw_speed)
                        #self.sim.roll(roll_speed)
                        #self.sim.pitch(pitch_speed)
                        
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
