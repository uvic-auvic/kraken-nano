import launch
import launch_ros.actions
from subprocess import Popen
import time

def generate_launch_description():
    Popen(["ign", "gazebo", "-r", "/home/ubuntu/Documents/uvic/kraken-nano/ROS/ws/src/kraken/worlds/kraken.sdf"])
    
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/F_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/B_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/L_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/R_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/FL_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/FR_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/BL_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/auv/joint/BR_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/altimeter@ros_gz_interfaces/msg/Altimeter[ignition.msgs.Altimeter"])
    Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"])
    
    time.sleep(10)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="kraken",
            executable="controller",
            name="controller"),
    ])
