import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64
import time

sys.path.append("/home/kraken/kraken-nano/ROS/ws/src/kraken/kraken/include")

#from simulation import Simulation
from motorboard import MotorBoard
from pid import PID
from serial import Serial

from custom.msg import PoseE

class spinMotor(Node):

        def __init__(self):
                super().__init__('spinMotor')
                
                #self.pid_timer = self.create_timer(pid_period, self.pid_callback)
                self.logger = self.get_logger()
                
                #self.sim = Simulation(self)
                
                #self.kill_switch = Serial("/dev/ttyTCU0", 115200, timeout=3)
                
                mb = MotorBoard("/dev/ttyTHS1")
                
                self.logger.info("Init motors")
                
                mb.init_motors()
                
                time.sleep(2)
                #time.sleep(20)
                mb.down()
                mb.left()
                mb.forward()
                mb.send_motors(40)
                time.sleep(1)
                mb.cut_motors()
                mb.right()
                mb.up()
                mb.backward()
                mb.send_motors(40)
                time.sleep(1)
                mb.cut_motors()

def main(args=None):
        rclpy.init(args=args)

        spinmotor = spinMotor() 

        rclpy.spin(spinmotor)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        spinmotor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
        main()
