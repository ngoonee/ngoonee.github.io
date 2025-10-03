---
title: Launch File
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 10
---

## Launch files

Launch files simplify the process of starting multiple nodes and configuring them. They are XML files that can be used to launch multiple nodes with a single command. In this section, we will create a launch file that will launch nodes in the "turtle_controller" package.

Create a new folder called "launch" in the root of the package. Create a new launch file named "turtle.launch.py" in the "turtle_controller" package and key in the following code:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create a LaunchDescription object to hold the collection of launch actions
    return LaunchDescription(
        [
            # Launch a node running the "turtlesim" package with the "turtlesim_node" executable
            Node(
                package="turtlesim",
                executable="turtlesim_node",
            ),
            # Launch a node running the "turtle_controller" package with the "turtle_sensor" executable
            Node(
                package="turtle_controller",
                executable="turtle_sensor",
            ),
            # Launch a node running the "turtle_controller" package with the "turtle_action_server" executable
            Node(
                package="turtle_controller",
                executable="turtle_action_server",
            ),
        ]
    )
```

Update setup.py to copy the launch files over when building. Add the line below in to the data_files array

```python
("share/" + package_name + "/launch", ["launch/turtle.launch.py"])
```

Run build again to copy the launch files over. Source after building and you should be able to launch the nodes with the following command:

```bash
ros2 launch turtle_controller turtle.launch.py 
```

![Launch files](/assets/images/ros/launch-file/launch.gif)

---

[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/action-server.md %})
