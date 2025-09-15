#!/usr/bin/env python3
"""
Alert Manager Node 
Subscribes to /health_status and manages logging/alerts
Basic functionality with rolling window context only
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from datetime import datetime


class AlertManagerNode(Node):
    def __init__(self):
        super().__init__('alert_manager')

        # ROS parameters
        self.declare_parameter('alert_cooldown_seconds', 10.0)
        
        self.alert_cooldown = self.get_parameter('alert_cooldown_seconds').value

        # Track last alert times and previous state
        self.last_alert_time = {
            'HEALTHY': 0,
            'WARNING': 0, 
            'CRITICAL': 0
        }
        self.previous_state = None

        # Subscribe to health status
        self.subscription = self.create_subscription(String, '/health_status', self.health_callback, 10)

        self.get_logger().info(f'Alert Manager started (cooldown: {self.alert_cooldown}s)')

    def health_callback(self, msg):
        """Process incoming health status messages"""
        try:
            # Parse the health status
            data = json.loads(msg.data)
            state = data['health_state']
            reason = data['reason']

            # Check if this is a state transition
            state_changed = (state != self.previous_state)

            # Check if enough time has passed since last alert of this type
            now = time.time()
            time_since_last = now - self.last_alert_time[state]
            can_send_alert = time_since_last > self.alert_cooldown

            # Send alerts based on conditions
            if state_changed or can_send_alert:
                history_data = data.get('history', {})
                self.send_alert(state, reason, state_changed, history_data)
                self.last_alert_time[state] = now

            # Update previous state
            self.previous_state = state

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in health status message")
        except KeyError as e:
            self.get_logger().error(f"Missing field in health status: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing health status: {e}")

    def send_alert(self, state, reason, is_transition, history_data):
        """Send appropriate alert based on health state with rolling window context"""
        context = self.build_context_string(history_data.get('summary', {})) if history_data else ""

        if state == 'HEALTHY':
            if is_transition:
                msg = f"✅ Robot status: HEALTHY - {reason}"
                if context:
                    msg += f" | {context}"
                self.get_logger().info(msg)
                
                if self.previous_state:
                    transition_msg = f"State transition: {self.previous_state} → HEALTHY"
                    self.get_logger().info(transition_msg)

        elif state == 'WARNING':
            msg = f"⚠️  WARNING: {reason}"
            if context:
                msg += f" | {context}"
            self.get_logger().warn(msg)
            
            if is_transition and self.previous_state:
                transition_msg = f"State transition: {self.previous_state} → WARNING"
                self.get_logger().warn(transition_msg)

        elif state == 'CRITICAL':
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            alert_msg = f"🚨 CRITICAL ALERT: {reason}"
            
            if context:
                alert_msg += f" | {context}"

            self.get_logger().error(alert_msg)
            print(f"[{timestamp}] {alert_msg}")

            if is_transition and self.previous_state:
                transition_msg = f"CRITICAL STATE TRANSITION: {self.previous_state} → CRITICAL"
                self.get_logger().error(transition_msg)
                print(f"[{timestamp}] {transition_msg}")

    def build_context_string(self, summary):
        """Build contextual information from rolling window summary"""
        if not summary:
            return ""

        context_parts = []
        
        # Battery context
        battery = summary.get('battery', {})
        if battery:
            avg_battery = battery.get('avg', 0)
            min_battery = battery.get('min', 0)
            max_battery = battery.get('max', 0)
            context_parts.append(f"Battery: {avg_battery:.1f}% avg ({min_battery:.1f}-{max_battery:.1f}%)")

        # Motor context
        motor1 = summary.get('motor1_rpm', {})
        motor2 = summary.get('motor2_rpm', {})
        if motor1 and motor2:
            m1_avg = motor1.get('avg', 0)
            m2_avg = motor2.get('avg', 0)
            context_parts.append(f"Motors: M1={m1_avg:.0f} M2={m2_avg:.0f} RPM avg")

        # Time span context
        time_span = summary.get('time_span', 0)
        if time_span > 0:
            context_parts.append(f"Window: {time_span:.1f}s")

        return " | ".join(context_parts)


def main():
    rclpy.init()
    node = AlertManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Alert Manager shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
