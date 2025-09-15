#!/usr/bin/env python3
"""
Health Analyzer Node
Monitors robot status and reports health condition
Maintains a rolling window of last 10 status readings
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import RobotStatus
import json
import time
from collections import deque


class HealthAnalyzerNode(Node):
    def __init__(self):
        super().__init__('health_analyzer')

        # Declare ROS parameters with default values
        self.declare_parameter('battery_healthy_threshold', 25.0)
        self.declare_parameter('battery_warning_threshold', 15.0)  
        self.declare_parameter('motor_timeout_seconds', 5.0)
        self.declare_parameter('publish_frequency', 5.0)
        self.declare_parameter('motor_rpm_min', -2000)
        self.declare_parameter('motor_rpm_max', 2000)
        self.declare_parameter('rolling_window_size', 10)

        # Get parameter values
        self.battery_healthy = self.get_parameter('battery_healthy_threshold').value
        self.battery_critical = self.get_parameter('battery_warning_threshold').value  
        self.motor_timeout = self.get_parameter('motor_timeout_seconds').value
        self.publish_rate = self.get_parameter('publish_frequency').value
        self.motor_min_rpm = self.get_parameter('motor_rpm_min').value
        self.motor_max_rpm = self.get_parameter('motor_rpm_max').value
        self.window_size = self.get_parameter('rolling_window_size').value

        # State tracking
        self.robot_data = None
        self.last_motor_time = time.time()
        
        # Rolling window for status history
        self.status_window = deque(maxlen=self.window_size)

        # ROS setup
        self.subscriber = self.create_subscription(RobotStatus, '/robot_status', self.robot_callback, 10)

        self.publisher = self.create_publisher(String, '/health_status', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.check_health)

        # Log startup info
        self.get_logger().info('Health Analyzer started with:')
        self.get_logger().info(f'  Battery thresholds: {self.battery_critical}% (critical) / {self.battery_healthy}% (healthy)')
        self.get_logger().info(f'  Motor RPM range: {self.motor_min_rpm} to {self.motor_max_rpm}')
        self.get_logger().info(f'  Motor timeout: {self.motor_timeout}s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate}Hz')
        self.get_logger().info(f'  Rolling window size: {self.window_size} readings')

    def robot_callback(self, msg):
        """Store new robot data and update motor activity time"""
        self.robot_data = msg

        # Update motor activity if either motor is moving
        if msg.motor1_rpm != 0 or msg.motor2_rpm != 0:
            self.last_motor_time = time.time()

        # Add current reading to rolling window
        current_reading = {
            'timestamp': time.time(),
            'battery_charge': msg.battery_charge,
            'motor1_rpm': msg.motor1_rpm,
            'motor2_rpm': msg.motor2_rpm,
            'bit_errors': msg.bit_errors
        }
        
        self.status_window.append(current_reading)

    def check_health(self):
        """Check robot health and publish status"""

        # No data = critical
        if self.robot_data is None:
            state = "CRITICAL"
            reason = "No robot data"
        else:
            state, reason = self.get_health_state()

        # Create health message
        health_info = {
            'health_state': state,
            'reason': reason,
            'timestamp': time.time(),
            'node': 'health_analyzer'  
        }

        # Add current details if we have robot data
        if self.robot_data is not None:
            health_info['details'] = {
                'battery_charge': self.robot_data.battery_charge,
                'motor1_rpm': self.robot_data.motor1_rpm,
                'motor2_rpm': self.robot_data.motor2_rpm,
                'bit_errors': self.robot_data.bit_errors
            }

        # Add rolling window data
        health_info['history'] = {
            'window_size': len(self.status_window),
            'readings': list(self.status_window),
            'summary': self.get_window_summary()
        }

        # Publish message
        msg = String()
        msg.data = json.dumps(health_info)
        self.publisher.publish(msg)

    def get_window_summary(self):
        """Generate summary statistics from the rolling window"""
        if not self.status_window:
            return {}

        # Extract values for analysis
        battery_values = [reading['battery_charge'] for reading in self.status_window]
        motor1_values = [reading['motor1_rpm'] for reading in self.status_window]
        motor2_values = [reading['motor2_rpm'] for reading in self.status_window]

        summary = {
            'battery': {
                'min': min(battery_values),
                'max': max(battery_values),
                'avg': sum(battery_values) / len(battery_values)
            },
            'motor1_rpm': {
                'min': min(motor1_values),
                'max': max(motor1_values),
                'avg': sum(motor1_values) / len(motor1_values)
            },
            'motor2_rpm': {
                'min': min(motor2_values),
                'max': max(motor2_values),
                'avg': sum(motor2_values) / len(motor2_values)
            },
            'time_span': self.status_window[-1]['timestamp'] - self.status_window[0]['timestamp'] if len(self.status_window) > 1 else 0
        }

        return summary

    def get_health_state(self):
        """Determine health state based on robot data"""
        data = self.robot_data

        # CRITICAL CONDITIONS

        # Battery too low
        if data.battery_charge < self.battery_critical:
            return "CRITICAL", f"Battery critical: {data.battery_charge:.2f}%"

        # Motors not responding
        motor_dead_time = time.time() - self.last_motor_time
        both_motors_stopped = (data.motor1_rpm == 0 and data.motor2_rpm == 0)

        if both_motors_stopped and motor_dead_time > self.motor_timeout:
            return "CRITICAL", f"Motors dead for {motor_dead_time:.2f}s"

        # WARNING CONDITIONS
        warnings = []

        # Low battery warning
        if self.battery_critical < data.battery_charge < self.battery_healthy:
            warnings.append(f"Battery low: {data.battery_charge:.2f}%")

        # Motor speed warnings  
        if not (self.motor_min_rpm <= data.motor1_rpm <= self.motor_max_rpm):
            warnings.append(f"Motor1 speed: {data.motor1_rpm}")

        if not (self.motor_min_rpm <= data.motor2_rpm <= self.motor_max_rpm):
            warnings.append(f"Motor2 speed: {data.motor2_rpm}")

        # Error warnings
        if data.bit_errors > 0:
            warnings.append(f"System errors: {data.bit_errors}")

        # Return warning if any issues found
        if warnings:
            return "WARNING", "; ".join(warnings)

        # Everything is good
        return "HEALTHY", "All systems good"


def main(args=None):
    rclpy.init(args=args)
    node = HealthAnalyzerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
