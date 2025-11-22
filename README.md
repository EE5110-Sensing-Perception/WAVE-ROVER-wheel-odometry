# odom_wheel

A ROS2 package for open-loop odometry estimation based on commanded velocity (`/cmd_vel`) for a 4-wheeled robot without wheel encoders.

## Overview

This package provides an odometry estimation node that calculates the robot's pose and velocity by integrating commanded velocities over time. It's designed for robots that don't have wheel encoders but can estimate their motion based on the commanded velocity commands.

## Features

- Open-loop odometry estimation from `/cmd_vel_executed` topic
- Publishes odometry data to `/odom_wheel` topic
- Broadcasts TF transform from `odom` to `base_link`
- Configurable robot parameters (wheel radius, wheel base, update rate)
- Experimental calibration support for velocity conversion

## Dependencies

- ROS2 (tested with Humble/Humble)
- `rclcpp` - C++ ROS2 client library
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types
- `tf2_ros` - TF2 transform library
- `launch` and `launch_ros` - Launch file support

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select odom_wheel
source install/setup.bash
```

## Usage

### Launch the odometry estimator

```bash
ros2 launch odom_wheel odometry_launch.py
```

### Run the node directly

```bash
ros2 run odom_wheel odom_wheel
```

### With custom parameters

```bash
ros2 run odom_wheel odometry_estimator --ros-args \
  -p wheel_radius:=0.05 \
  -p wheel_base:=0.13 \
  -p update_rate:=30.0
```

## Topics

### Subscribed Topics

- `/cmd_vel_executed` (`geometry_msgs/msg/Twist`) - The executed velocity command (scaled 0-1.0)

### Published Topics

- `/odom_wheel` (`nav_msgs/msg/Odometry`) - Estimated odometry data

### TF Frames

- `odom` → `base_link` - Transform from odometry frame to base link

## Parameters

The node accepts the following parameters (can be set via launch file or command line):

- `wheel_radius` (double, default: 0.08 m) - Radius of the wheels
- `wheel_base` (double, default: 0.30 m) - Distance between left and right wheels
- `update_rate` (double, default: 20.0 Hz) - Rate at which odometry is updated and published
- `rotation_time_at_half_vel` (double, default: 0.375 s) - Time for one wheel rotation at cmd_vel = 0.5 (for calibration)
- `cmd_vel_test_value` (double, default: 0.5) - cmd_vel value used during calibration (0-1.0 scale)

Parameters can be configured in `config/robot_params.yaml` or passed via launch file arguments.

## Configuration

Edit `config/robot_params.yaml` to adjust robot-specific parameters:

```yaml
odometry_estimator:
  ros__parameters:
    wheel_radius: 0.04        # meters
    wheel_base: 0.30          # meters
    update_rate: 20.0         # Hz
    rotation_time_at_half_vel: 0.375  # seconds
    cmd_vel_test_value: 0.5
```

## How It Works

1. The node subscribes to `/cmd_vel_executed` topic which contains commanded linear and angular velocities.
2. It converts the commanded velocities to left and right wheel velocities using the wheel base.
3. A conversion factor (calculated from experimental calibration) scales the cmd_vel values to actual wheel velocities.
4. The pose is integrated over time using the wheel velocities.
5. Odometry and TF transforms are published at the configured update rate.

## Calibrating rotation time
Because there's no encoders in the motors, we need to do a calibration to estimate the distance traveled. To do this, we 
measure the time taken for one wheel rotation, ideally at 0.5 speed. This allows us to estimate speed and distance traveled.
A simple procedure is:
- put the robot on blocks, so the wheels can rotate freely
- set the /cmd_vel speed to 0.5, and record video of the wheel rotating
- Measure the time taken for one complete rotation (taking multiple measurements and averaging helps). 
- Set the rotation_time_at_half_vel in the config yaml file to the measured speed

You can validate the odometry by driving the car in a square 1m pattern, and check the difference between the odometry position and actual robot position.

## Limitations

- This is an **open-loop** odometry system - it does not use sensor feedback
- Accuracy degrades over time due to integration drift
- Wheel slip and other disturbances are not accounted for
- For better accuracy, consider using wheel encoders or other sensors

## License

MIT License - see LICENSE file for details.

## Author

Maintained by Brian Deegan (brian.deegan82@gmail.com)

