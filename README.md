# ros_networktables_bridge_host

ros_networktables_bridge_host is a ROS package that bridges ROS (Robot Operating System) to the roboRIO using NetworkTables and JSON formatted NT entries. This project aims to provide a seamless integration between FRC (FIRST Robotics Competition) robots using WPILib and ROS, enabling advanced robot control, sensor integration, and autonomy. It's a cousin project to the Java client <https://github.com/frc-88/ROSNetworkTablesBridge>

**This version has been converted to the appropriate ROS2 syntax but has not been thoroughly tested. Original ROS version created by FRC 88 (TJ^2).**

# Features

- Connects WPILib-based FRC robots with ROS using NetworkTables
- Uses JSON formatted NT entries for standardized communication
- Supports custom and standard ROS messages

# Prerequisites

- ROS environment setup (tested with ROS Kilted)
- rosbridge_library
- pynetworktables version 2021.0.0

# Installation

## rosbridge_suite

### From apt

- run `sudo apt-get install ros-noetic-rosbridge-suite`

### From source

- Clone it into your workspace:

```bash
git clone https://github.com/RobotWebTools/rosbridge_suite.git](https://github.com/FRC900/ros2_networktables_bridge_host.git
```

- Build and source:

```bash
colcon build --symlink-install && source install/setup.bash
```

# Example usage

- Create a launch file:

```xml
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nt_client = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_networktables_bridge_host'),
                'launch',
                'nt_client.launch.py'
            ])
        ),
        launch_arguments={'nt_host': '10.0.88.2'}.items()
    )

    return LaunchDescription([nt_client])
```

- This is an example where the team number is `88`

- Launch the node: `ros2 launch path/to/your/file.launch.py`

# Contributing

Contributions are welcome! Please follow the guidelines in the CONTRIBUTING.md file in <https://github.com/frc-88/ROSNetworkTablesBridge>
