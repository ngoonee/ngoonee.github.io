---
title: Setup
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 0
---

## Virtualisation

Four primary options for operating system:-
  * Bare-metal - Fully wipe existing operating system to install Ubuntu
  * Dual-boot - Install Ubuntu on another partition or disc
  * Virtualisation - Create a virtual disc and virtual machine for Ubuntu (Easiest to use [Virtualbox](https://www.virtualbox.org/wiki/Downloads){:target="_blank"})
  * WSL2 - install a pre-made virtual machine which runs Ubuntu

|                    | Bare-metal                                         | Dual-boot                                                 | Virtualisation                   | WSL2                                       |
|--------------------|----------------------------------------------------|-----------------------------------------------------------|----------------------------------|--------------------------------------------|
| Performance        | Best                                               | Best                                                      | Moderate                         | Moderate                                   |
| Ease of setup      | Involved (many steps)                              | Complex (many steps, and may accidentally delete Windows) | Easy              | Easy                                           |
| Resource usage     | Efficient                                          | Efficient (duplicate storage usage)                       | Heavy                            | Mostly efficient                           |
| Hardware access    | Full                                               | Full                                                      | Limited                          | Less limited but finicky                   |
| Networking         | Native                                             | Native                                                    | Bridged (complex)                | Mildly complex                             |
| ROS2 compatability | Best                                               | Best                                                      | Very good                        | Good (software), unreliable (hardware)     |
| RViz/Gazebo        | Excellent                                          | Excellent                                                 | Slow                             | Not very fast                              |
| Long term use      | Stable                                             | Less stable (bootloader issues on update)                 | Easy (snapshots/recreate)        | Easy                                       |
| Main advantage     | Full performance and hardware access               | Full performance and hardware access                      | Easy setup, isolated environment | Easiest setup, mostly isolated environment |
| Main disadvantages | No more Windows OS, not possible on Apple machines | Not possible on Apple machines, Windows update issues     | Lower performance                | Less popular/well-supported                |

In general, start with virtualisation (low commitment). Move to dual-boot or bare-metal if performance and/or networking becomes an issue. Be very careful when handling filesystems and partitions.

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
