---
title: Setup
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 0
---

## Operating System Setup

Follow the instructions on the [Ubuntu website](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview){:target="_blank"} to install Ubuntu 22.04. This specific version of Ubuntu is needed for ROS 2 Humble. Each release of ROS 2 is tied to a specific version of Ubuntu. The [ROS 2 Humble](https://docs.ros.org/en/humble/index.html){:target="_blank"} documentation provides more information about the release.

## ROS 2 Installation

Follow the instructions on the [ROS 2 Debian Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html){:target="_blank"} documentation to install ROS 2 Humble. Follow the instructions provided to setup the ROS 2 apt repository and install the ROS 2 packages. Run the Desktop Install (recommended) instructions for the installation.

Ensure that you can run the example in the setup instructions.

## Colcon Installation

Colcon is a command line tool that builds packages in a workspace. It is the recommended build tool for ROS 2. Use the instructions provided to install Colcon.

```bash
sudo apt install python3-colcon-common-extensions
```

Refer to [Using colcon to build packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html){:target="_blank"} for the full document.

## VS Code Installation

VScode will be the main IDE for the development. Install VScode by following the instructions on the [VScode website](https://code.visualstudio.com/docs/setup/linux){:target="_blank"}.

---
[Next]({{ site.baseurl }}{% link robot-operating-system-basics/key-concepts.md %})
