---
title: Gazebo Garden
layout: default
parent: Robot Operating System (ROS) Simulation
nav_order: 1
---

## Gazebo Garden

Launch Gazebo with the following command:

```bash
gz sim
```

![Gazebo Garden Loading Screen](/assets/images/ros-simulation/gazebo/garden.png)

Ensure tha the version shown is v7.5.0 Garden. Select Tugbot in warehouse and run the simulation. For first launch gazebo will take sometime to download all the necessary assets. Once the simulation is running you should see the following screen:

![Gazebo Garden Simulation Screen](/assets/images/ros-simulation/gazebo/tugbot.png)

Familiarize your self with the interface of Gazebo Garden. You can move the robot around using the build in teleop tab. The entity tab allows you to inspect all the simulated elements in Gazebo.

![Gazebo Garden Entity](/assets/images/ros-simulation/gazebo/entity.png)

---
[Next]({{ site.baseurl }}{% link ros-simulation/ros2-integration.md %})
