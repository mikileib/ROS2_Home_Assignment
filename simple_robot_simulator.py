#!/usr/bin/env python3
"""
Simple Robot Simulator

A basic robot status simulator for testing health monitoring.
Can be run with different preset scenarios.
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
import random


class SimpleRobotSimulator(Node):
    """Simple robot simulator with predefined scenarios."""

    def __init__(self, scenario="normal"):
        super().__init__('simple_robot_simulator')

        self.scenario = scenario
        self.battery_level = 100.0
        self.time_elapsed = 0.0

        # Publisher
        self.publisher = self.create_publisher(RobotStatus, '/robot_status', 10)

        # Timer - publish at 5Hz
        self.timer = self.create_timer(0.2, self.publish_status)

        self.get_logger().info(f'Simple Robot Simulator started with scenario: {scenario}')

    def publish_status(self):
        """Publish robot status based on selected scenario."""
        msg = RobotStatus()
        self.time_elapsed += 0.2

        if self.scenario == "normal":
            msg.battery_charge = 80.0 + random.uniform(-5, 5)
            msg.motor1_rpm = 1500 + random.randint(-100, 100)
            msg.motor2_rpm = 1450 + random.randint(-100, 100)
            msg.bit_errors = 0

        elif self.scenario == "low_battery":
            msg.battery_charge = 20.0 + random.uniform(-2, 2)
            msg.motor1_rpm = 1500 + random.randint(-100, 100)
            msg.motor2_rpm = 1450 + random.randint(-100, 100)
            msg.bit_errors = 0

        elif self.scenario == "critical_battery":
            msg.battery_charge = 10.0 + random.uniform(-2, 2)
            msg.motor1_rpm = 1200 + random.randint(-200, 200)
            msg.motor2_rpm = 1150 + random.randint(-200, 200)
            msg.bit_errors = 1

        elif self.scenario == "motor_failure":
            msg.battery_charge = 50.0 + random.uniform(-5, 5)
            # Motors become unresponsive after 3 seconds
            if self.time_elapsed > 3.0:
                msg.motor1_rpm = 0
                msg.motor2_rpm = 0
            else:
                msg.motor1_rpm = 1500 + random.randint(-100, 100)
                msg.motor2_rpm = 1450 + random.randint(-100, 100)
            msg.bit_errors = 0

        elif self.scenario == "bit_errors":
            msg.battery_charge = 60.0 + random.uniform(-5, 5)
            msg.motor1_rpm = 1500 + random.randint(-100, 100)
            msg.motor2_rpm = 1450 + random.randint(-100, 100)
            msg.bit_errors = random.randint(1, 3)

        elif self.scenario == "rpm_out_of_range":
            msg.battery_charge = 70.0 + random.uniform(-5, 5)
            msg.motor1_rpm = 2500 + random.randint(-100, 100)  # Out of range
            msg.motor2_rpm = -2200 + random.randint(-100, 100)  # Out of range
            msg.bit_errors = 0

        elif self.scenario == "emergency":
            # Multiple failures
            self.battery_level -= 0.5  # Rapid drain
            msg.battery_charge = max(5.0, self.battery_level)
            msg.motor1_rpm = 0
            msg.motor2_rpm = 0
            msg.bit_errors = 5

        else:  # Default to normal
            msg.battery_charge = 80.0
            msg.motor1_rpm = 1500
            msg.motor2_rpm = 1450
            msg.bit_errors = 0

        # Ensure values are within reasonable bounds
        msg.battery_charge = max(0.0, min(100.0, msg.battery_charge))
        msg.motor1_rpm = max(-3000, min(3000, msg.motor1_rpm))
        msg.motor2_rpm = max(-3000, min(3000, msg.motor2_rpm))
        msg.bit_errors = max(0, msg.bit_errors)

        self.publisher.publish(msg)

        # Log every 5 seconds
        if int(self.time_elapsed * 5) % 25 == 0:
            self.get_logger().info(
                f'Publishing: Battery={msg.battery_charge:.1f}%, '
                f'Motors={msg.motor1_rpm}/{msg.motor2_rpm}, '
                f'BIT={msg.bit_errors}'
            )


def main():
    """Main function with scenario selection."""
    import sys

    # Get scenario from command line argument
    scenario = "normal"
    if len(sys.argv) > 1:
        scenario = sys.argv[1]

    print(f"Starting Simple Robot Simulator with scenario: {scenario}")
    print("Available scenarios:")
    print("  normal - Normal operation (default)")
    print("  low_battery - Low battery warning")
    print("  critical_battery - Critical battery")
    print("  motor_failure - Motors stop after 3 seconds")
    print("  bit_errors - Random BIT errors")
    print("  rpm_out_of_range - Motors RPM out of range")
    print("  emergency - Multiple system failures")
    print("")
    print("Usage: python3 simple_robot_simulator.py [scenario]")
    print("Press Ctrl+C to stop")
    print("")

    rclpy.init()

    try:
        simulator = SimpleRobotSimulator(scenario)
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        print("\nSimulator stopped")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
