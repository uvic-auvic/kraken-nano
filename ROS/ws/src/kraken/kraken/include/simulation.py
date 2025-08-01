from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Altimeter

# Creates publishers for each thruster and publishes values based on desired actions
class Simulation():
	def __init__(self, node):
		self.FP = node.create_publisher(Float64, '/model/auv/joint/F_joint/cmd_thrust', 10) # Front
		self.BP = node.create_publisher(Float64, '/model/auv/joint/B_joint/cmd_thrust', 10) # Back
		self.LP = node.create_publisher(Float64, '/model/auv/joint/L_joint/cmd_thrust', 10) # Left
		self.RP = node.create_publisher(Float64, '/model/auv/joint/R_joint/cmd_thrust', 10) # Right
		self.FLP = node.create_publisher(Float64, '/model/auv/joint/FL_joint/cmd_thrust', 10) # Front Left
		self.FRP = node.create_publisher(Float64, '/model/auv/joint/FR_joint/cmd_thrust', 10) # Front Right
		self.BLP = node.create_publisher(Float64, '/model/auv/joint/BL_joint/cmd_thrust', 10) # Back Left
		self.BRP = node.create_publisher(Float64, '/model/auv/joint/BR_joint/cmd_thrust', 10) # Back Right
		self.IMU_sub = node.create_subscription(Imu, '/imu', self.imu_callback, 10)
		self.Altimeter_sub = node.create_subscription(Altimeter, '/altimeter', self.altimeter_callback, 10)
		self.imu = None
		self.altimeter = None
		
	# Move the simulated AUV forward
	def forward(self, speed):
		msg = Float64()
		msg.data = float(speed)
		self.LP.publish(msg)
		self.RP.publish(msg)
	
	# Move the simulated AUV backward
	def backward(self, speed):
		msg = Float64()
		msg.data = float(-speed)
		self.LP.publish(msg)
		self.RP.publish(msg)
		
	# Move the simulated AUV left
	def left(self, speed):
		msg = Float64()
		msg.data = float(speed)
		self.FP.publish(msg)
		self.BP.publish(msg)
	
	# Move the simulated AUV right
	def right(self, speed):
		msg = Float64()
		msg.data = float(-speed)
		self.FP.publish(msg)
		self.BP.publish(msg)
	
	# Move the simulated AUV up
	def up(self, speed):
		msg = Float64()
		msg.data = float(speed)
		self.FLP.publish(msg)
		self.FRP.publish(msg)
		self.BLP.publish(msg)
		self.BRP.publish(msg)
	
	# Move the simulated AUV down
	def down(self, speed):
		msg = Float64()
		msg.data = float(-speed)
		self.FLP.publish(msg)
		self.FRP.publish(msg)
		self.BLP.publish(msg)
		self.BRP.publish(msg)
		
	def get_imu(self):
		return self.imu
	
	def get_altimeter(self):
		return self.altimeter
		
	def imu_callback(self, msg):
		self.imu = msg
	
	def altimeter_callback(self, msg):
		self.altimeter = msg
