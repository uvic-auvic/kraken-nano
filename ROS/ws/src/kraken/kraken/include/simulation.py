from std_msgs.msg import Float64

class Simulation():
	def __init__(self, node):
		self.FP = node.create_publisher(Float64, '/model/auv/joint/F_joint/cmd_thrust', 10)
		self.BP = node.create_publisher(Float64, '/model/auv/joint/B_joint/cmd_thrust', 10)
		self.LP = node.create_publisher(Float64, '/model/auv/joint/L_joint/cmd_thrust', 10)
		self.RP = node.create_publisher(Float64, '/model/auv/joint/R_joint/cmd_thrust', 10)
		self.FLP = node.create_publisher(Float64, '/model/auv/joint/FL_joint/cmd_thrust', 10)
		self.FRP = node.create_publisher(Float64, '/model/auv/joint/FR_joint/cmd_thrust', 10)
		self.BLP = node.create_publisher(Float64, '/model/auv/joint/BL_joint/cmd_thrust', 10)
		self.BRP = node.create_publisher(Float64, '/model/auv/joint/BR_joint/cmd_thrust', 10)
		
	def left(self, speed):
		msg = Float64()
		msg.data = float(speed)
		self.FP.publish(msg)
		self.BP.publish(msg)
