---
title: ROS Project
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 3
---

## ROS Project

This section will cover the development of a ROS project. It will explain the steps involved in creating the projects. We will begin by creating a ROS workspace.

## Setup Tools

To ensure compatibility with ROS2 Python packages and eliminate warnings, it is recommended to use setup tools version 58.2.0, the last version known to work seamlessly. If your current version of setup tools is higher than 58.2.0, you will need to downgrade it. Execute the following command to perform the downgrade:

```bash
pip3 install setuptools==58.2.0
```

For more information about this solution, you can refer to [this link](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/#400052){: target="_blank"}.

## ROS Workspace

If you have created the workspace, you can skip the steps below. Otherwise, follow the steps below to create a ROS workspace.

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
# Change directory to the workspace
cd ~/ros2_ws
```

Build the workspace.

```bash
# Build the workspace
colcon build --symlink-install
```

After building the workspace, you will be able to see the folders shown in the image below.

![Workspace Directory](/assets/images/ros/project/files.png)

Once succesfully build, there will be 4 workspace directories:

- build
- install
- log
- src

The "src" directory, which we created earlier, is where you will create your ROS packages. The "build" directory is where the build files will be stored. The "install" directory is where the installed files will be stored. Lastly, the "log" directory is where the log files will be stored.

## ROS Package

Let's create a ROS package called "turtle_controller". Make sure that you are in the source folder when running the command below. To create the package, run the following command:

```bash
# Create ros2 package turtle_controller with ament_python build type and node name turtle_driver
ros2 pkg create --build-type ament_python --node-name turtle_driver turtle_controller
```

The command above will create package name **"turtle_controller"** with a node named **"turtle_driver"**. Go to the folder and open it with vscode using the command below.

```bash
# Change directory to the package
cd ~/ros2_ws/src/turtle_controller
# Open the package with vscode
code .
```

The files and folder structure of the package is shown in the image below.

![Workspace Directory](/assets/images/ros/project/package.png)

In the **package.xml** and **setup.py** files, you will find certain information that needs to be updated. Look for the sections marked with **TODO** in both files and make the necessary changes to those sections.

![TODO changes](/assets/images/ros/project/todo.png)

Once you have made the necessary changes, you can build the package by using the following command:

```bash
# Change directory to the workspace
cd ~/ros2_ws
# Build the workspace
colcon build --symlink-install
```

After building the package, you need to source the workspace. To do this, execute the following command:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash
```

After sourcing the workspace, you can run the node using the following command:

```bash
# Run the node
ros2 run turtle_controller turtle_driver
```

You will see the output shown in the image below.

![TODO changes](/assets/images/ros/project/hi_world.png)

---
[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/ros-basics.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/publisher.md %})
