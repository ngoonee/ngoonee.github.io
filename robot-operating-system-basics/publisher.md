---
title: Publisher
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 4
---

## Publisher

In this section, we will create a publisher node that publishes a message to the topic "turtle1/cmd_vel". The node, named "turtle_driver", will be part of the project we created in the previous section. Open the file "turtle_driver.py" in the "turtle_controller" folder of the "turtle_controller" package and key in the following code:

```python
import rclpy  # Import the ROS 2 Python client library
import math  # Import the math module for mathematical operations

from rclpy.node import Node  # Import the Node class from rclpy
from geometry_msgs.msg import Twist  # Import the Twist message type from geometry_msgs


# Define a SpeedPublisher class that extends the Node class
class SpeedPublisher(Node):
    # Define a list of speed values
    speed = [
        [1.0, 0.0],
        [1.0, 0.0],
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, math.pi / 2],
        [0.0, 0.0],
    ]

    # Define the constructor of the SpeedPublisher class
    def __init__(self):
        super().__init__("speed_publisher")  # Call the constructor of the Node class
        # Create a publisher for the "/turtle1/cmd_vel" topic
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        timer_period = 1  # Set the timer period to 1 second
        # Create a timer with the specified period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speedIndex = 0  # Initialize the speed index to 0

    # Define the timer callback function
    def timer_callback(self):
        msg = Twist()  # Create a new Twist message
        # Set the linear and angular velocity based on the current speed index
        msg.linear.x = self.speed[self.speedIndex][0]
        msg.angular.z = self.speed[self.speedIndex][1]
        # Publish the Twist message to the "/turtle1/cmd_vel" topic
        self.publisher_.publish(msg)
        self.speedIndex += 1  # Increment the speed index for the next iteration
        # Reset the speed index to 0 if it exceeds the maximum index
        if self.speedIndex == len(self.speed):
            self.speedIndex = 0
        # Log the published speed values
        self.get_logger().info(
            f"Publishing speed: x: {msg.linear.x}, z: {msg.angular.z}"
        )


# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    speed_publisher = SpeedPublisher()  # Create an instance of the SpeedPublisher class
    rclpy.spin(speed_publisher)  # Spin the node until it is shut down
    speed_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == "__main__":
    main()  # Call the main function when the script is executed directly
```

If you build your project using **colcon build --symlink-install**, you will not need to build again. Files that do not need to be compile such as Python scripts will be symlinked to the install folder. If you build your project using **colcon build**, you will need to build again.

To test out the create node, start turtlesim using the following command:

```bash
ros2 run turtlesim turtlesim_node
```

Run the node using the following command in another terminal:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash
ros2 run turtle_controller turtle_driver
```

You should see the turtle moving once you run the node.

![Turtle motion](/assets/images/ros/publisher/turtle.gif)

## Things to try

1. Change the way the speed values are defined and observe the turtle's motion.
2. Change the timer period and observe the turtle's motion.

---

[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/project.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/subscriber.md %})
