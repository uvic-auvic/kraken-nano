#!/usr/bin/env python3
"""
Simple test script to demonstrate planner motor control functionality.
This script shows how to manually trigger different movement tasks.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class PlannerTest(Node):
    def __init__(self):
        super().__init__('planner_test')
        
        # Publisher to send motor commands directly (for testing)
        self.motor_command_publisher = self.create_publisher(String, '/motor/command', 10)
        
        self.get_logger().info('Planner Test Node initialized')

    def send_test_command(self, command: str, speed: int, duration: float = 2.0):
        """Send a test motor command"""
        motor_msg = {
            'command': command,
            'speed': speed,
            'timestamp': str(self.get_clock().now().nanoseconds)
        }
        
        message = String()
        message.data = json.dumps(motor_msg)
        self.motor_command_publisher.publish(message)
        
        self.get_logger().info(f'Sent test command: {command} at speed {speed}')
        time.sleep(duration)
        
        # Send stop command
        stop_msg = {
            'command': 'stop',
            'speed': 0,
            'timestamp': str(self.get_clock().now().nanoseconds)
        }
        stop_message = String()
        stop_message.data = json.dumps(stop_msg)
        self.motor_command_publisher.publish(stop_message)
        self.get_logger().info('Sent stop command')

    def run_test_sequence(self):
        """Run a test sequence of motor commands"""
        self.get_logger().info('Starting motor test sequence...')
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Test sequence
        commands = [
            ("down", 20, 1.0),      # Dive down
            ("forward", 20, 1.0),   # Move forward
            ("left", 20, 1.0),      # Move left
            ("right", 20, 1.0),     # Move right
            ("yaw_cw", 20, 1.0),    # Rotate clockwise
            ("yaw_ccw", 20, 1.0),   # Rotate counter-clockwise
            ("up", 20, 1.0),        # Surface
        ]
        
        for command, speed, duration in commands:
            self.send_test_command(command, speed, duration)
            time.sleep(1.0)  # Pause between commands
        
        self.get_logger().info('Test sequence completed!')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = PlannerTest()
    
    try:
        # Run the test sequence
        test_node.run_test_sequence()
        
        # Keep node alive for a bit
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
