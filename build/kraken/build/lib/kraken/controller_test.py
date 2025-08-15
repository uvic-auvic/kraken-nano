#!/usr/bin/env python3
"""
Simple test script to send motor commands to the controller.
This script demonstrates how the planner sends commands to the controller.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ControllerTest(Node):
    def __init__(self):
        super().__init__('controller_test')
        
        # Publisher to send motor commands to controller
        self.motor_command_publisher = self.create_publisher(String, '/motor/command', 10)
        
        self.get_logger().info('Controller Test Node initialized')

    def send_motor_command(self, command: str, speed: int):
        """Send a motor command to the controller"""
        motor_msg = {
            'command': command,
            'speed': speed,
            'timestamp': str(self.get_clock().now().nanoseconds)
        }
        
        message = String()
        message.data = json.dumps(motor_msg)
        self.motor_command_publisher.publish(message)
        
        self.get_logger().info(f'Sent motor command: {command} at speed {speed}')

    def run_test_sequence(self):
        """Run a test sequence of motor commands"""
        self.get_logger().info('Starting controller test sequence...')
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Test sequence
        test_commands = [
            ("down", 30),       # Dive down
            ("forward", 30),    # Move forward
            ("stop", 0),        # Stop
            ("left", 30),       # Move left
            ("stop", 0),        # Stop
            ("right", 30),      # Move right
            ("stop", 0),        # Stop
            ("yaw_cw", 30),     # Rotate clockwise
            ("stop", 0),        # Stop
            ("yaw_ccw", 30),    # Rotate counter-clockwise
            ("stop", 0),        # Stop
            ("up", 30),         # Surface
            ("stop", 0),        # Final stop
        ]
        
        for command, speed in test_commands:
            self.send_motor_command(command, speed)
            time.sleep(1.0)  # Wait 1 second between commands

        self.get_logger().info('Controller test sequence completed!')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ControllerTest()
    
    try:
        # Run the test sequence
        test_node.run_test_sequence()
        
        # Keep node alive for a bit
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Controller test interrupted')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
