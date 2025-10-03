---
title: Key Concepts for ROS developments
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 1
---

## Key Concepts for ROS developments

To effectively develop with ROS (Robot Operating System) on Ubuntu, it's important to understand key concepts and tools that facilitate the development process.

## Using the terminal in Ubuntu

Mastering the usage of the terminal is crucial for effectively working with Ubuntu and ROS 2. The terminal, also known as the command line interface, allows you to interact with the operating system and execute commands. Here are some key concepts related to using the terminal:

![Ubuntu Terminal](/assets/images/ros/key-concepts/terminal.png)

### Navigating the File System

Ubuntu, like in most Linux-based operating systems, the file system follows a hierarchical structure. The file system is organized into a tree-like structure, starting from the root directory ("/") and branching out into subdirectories. Familiarize yourself with essential commands like cd (change directory) and ls (list files and directories). They enable you to navigate through different directories and access crucial files and folders, including your ROS 2 workspace.

Use the command below to change directory.

```bash
# Navigate to the download directory
cd ~/Downloads
```

The command below will list the contents of the current directory.

```bash
# List contents of download directory
ls
```

![Ubuntu Terminal](/assets/images/ros/key-concepts/cdls.png)

In the terminal, the symbols ~, ., and .. have specific meanings related to navigating the file system:

**~** In the context of the terminal, the tilde (~) represents the home directory of the current user. It is a shorthand notation that allows you to refer to your home directory without explicitly typing out the full path. For example, cd ~ would take you to your home directory regardless of your current location.

**.** The dot (.) represents the current directory. When used in a file path, it refers to the directory you are currently in. For example, if you are in the /home/user/documents directory and you execute a command like ls ., it will list the contents of the current directory (/home/user/documents).

**..** The double dot (..) represents the parent directory. It allows you to refer to the directory one level above the current directory. For example, if you are in the /home/user/documents directory and you execute a command like ls .., it will list the contents of the parent directory (/home/user).

The comand below will navigate to the parent directory.

```bash
# Navigate to the previous directory
cd ..
```

### Command Execution

The terminal allows you to execute various commands to compile code, launch ROS nodes, and manage packages. Understand the syntax and options of different commands to efficiently perform tasks in Ubuntu and ROS 2. The command **cd** and **ls** are examples of commands that you can execute in the terminal.

---
[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/setup.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/ros-basics.md %})
