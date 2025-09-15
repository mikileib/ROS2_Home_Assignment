# Health Monitor Package

A ROS2 robot health monitoring system that analyzes robot status data and provides real-time health classification with intelligent alerting.

## Overview

The health monitor analyzes robot status data and classifies overall health into HEALTHY, WARNING, or CRITICAL states with intelligent alerting and rolling window context.

**Components:**
- **Health Analyzer Node**: Processes robot status data, maintains rolling window of last 10 readings, publishes health status at 5Hz
- **Alert Manager Node**: Manages logging with appropriate ROS2 levels, implements alert frequency limiting, includes rolling window context

## Package Structure

```
health_monitor/
├── package.xml
├── setup.py  
├── health_monitor/
│   ├── health_analyzer_node.py   # Main health analysis with rolling window
│   └── alert_manager_node.py     # Alert management with context logging
├── launch/
│   └── health_monitor.launch.py
└── config/
    └── health_monitor_config.yaml
```

## Health Classification

- **HEALTHY**: Battery > 25%, motors within -2000 to 2000 RPM, no errors
- **WARNING**: Battery 15-25% OR motor RPM out of range OR any errors
- **CRITICAL**: Battery < 15% OR both motors unresponsive > 5 seconds

## Build and Run

### Prerequisites
- ROS2 Humble or later
- `interfaces` package with `RobotStatus` message
- Docker (for macOS users)

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select health_monitor
source install/setup.bash
```

### Run
```bash
# Complete system
ros2 launch health_monitor health_monitor.launch.py

# With robot data source
ros2 run ros2can can_ros2_status_node  # Real robot
# OR
python3 simple_robot_simulator.py normal  # Simulator
```

## Robot Simulator

Standalone Python script for testing:

```bash
# Available scenarios
python3 simple_robot_simulator.py normal           # Default
python3 simple_robot_simulator.py low_battery      # Warning condition  
python3 simple_robot_simulator.py critical_battery # Critical condition
python3 simple_robot_simulator.py motor_failure    # Motors stop after 3s
python3 simple_robot_simulator.py bit_errors       # Random errors
python3 simple_robot_simulator.py emergency        # Multiple failures
```

## Topics

- **Subscribes**: `/robot_status` (interfaces/RobotStatus)
- **Publishes**: `/health_status` (std_msgs/String) - JSON with health state, reason, and rolling window context

## Example Output

```bash
[INFO] [health_analyzer]: ✅ Robot status: HEALTHY - All systems good | Battery: 82.3% avg (80.1-84.5%) | Motors: M1=1502 M2=1448 RPM avg | Window: 2.0s

[WARN] [alert_manager]: ⚠️  WARNING: Battery low: 18.75% | Battery: 18.9% avg (17.2-20.1%) | Motors: M1=1480 M2=1445 RPM avg | Window: 2.0s

[ERROR] [alert_manager]: 🚨 CRITICAL ALERT: Battery critical: 12.34% | Battery: 13.1% avg (11.8-14.2%) | Motors: M1=1200 M2=1150 RPM avg | Window: 2.0s
```

## Testing

```bash
# Test with simulator
python3 simple_robot_simulator.py critical_battery
ros2 launch health_monitor health_monitor.launch.py

# Monitor output
ros2 topic echo /health_status
```

## Configuration

Key parameters in `health_monitor_config.yaml`:
- `battery_healthy_threshold`: 25.0% (default)
- `battery_warning_threshold`: 15.0% (default)  
- `motor_timeout_seconds`: 5.0 (default)
- `rolling_window_size`: 10 (default)
- `alert_cooldown_seconds`: 10.0 (default)