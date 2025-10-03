---
title: ROS 2 Integration
layout: default
parent: Robot Operating System (ROS) Simulation
nav_order: 2
---

## Gazebo Garden - ROS 2 Integration

Gazabo Garden provides a set of ROS 2 packages to integrate Gazebo with ROS 2. The integration is based on the Gazebo Transport library, which is a set of messages and services that enable the exchange of messages between Gazebo and ROS 2.

Check out the links below to learn more about the integration.

- [ROS 2 Integration](https://gazebosim.org/docs/garden/ros2_integration)
- [Gazebo transport](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-gazebo-transport-talker-and-ros-2-listener)

ros_gz_bridge provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo Transport. Its support is limited to only certain message types. Please, check this README to verify if your message type is supported by the bridge.

## Gazebo Garden - Topics & Services

Gazebo Transport is a publish-subscribe message passing system. It is designed to be efficient and easy to use. The transport system is based on Google's Protocol Buffers, a language-neutral, platform-neutral, extensible mechanism for serializing structured data. It allows for fast and efficient asynchronous message passing, services, and data logging.

You can list the topics using the following command:

```bash
gz topic -l
```

You can list the services using the following command:

```bash
gz service -l
```

## Gazebo Garden - Example

Refer to the link [https://github.com/dannyngweekiat/Gazebo-Garden-Simulation-Example](https://github.com/dannyngweekiat/Gazebo-Garden-Simulation-Example) for nav and slam example.

---
[Prev]({{ site.baseurl }}{% link ros-simulation/gazebo.md %})
