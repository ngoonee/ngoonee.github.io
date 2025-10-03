---
title: Subscriber
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 5
---

## Subscriber

In this section we are going to create a subscriber node that subscribes to the topic "turtle1/color_sensor". Create a new file named "turtle_sensor.py" in the "turtle_controller" folder of the "turtle_controller" package and key in the following code:

```python
import rclpy  # Import the ROS 2 Python client library

from rclpy.node import Node  # Import the Node class from rclpy
from turtlesim.msg import Color  # Import the Color message type from turtlesim.msg


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")  # Call the constructor of the Node class
        self.subscription = self.create_subscription(
            Color, "/turtle1/color_sensor", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Color):
        # Log the received color values
        self.get_logger().info(f"Color: r: {msg.r}, g: {msg.g}, b: {msg.b}")


# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    speed_publisher = (
        SensorSubscriber()
    )  # Create an instance of the SensorSubscriber class
    rclpy.spin(speed_publisher)  # Spin the node until it is shut down
    speed_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == "__main__":
    main()  # Call the main function when the script is executed directly
```

In the **setup.py** file, add the following line to the **"console_scripts"** array under the **"entry_points"** section:

```python
"turtle_sensor = turtle_controller.turtle_sensor:main"
```

Run the following command to build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller --symlink-install
```

Run the following command to launch the turtle_sensor node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtle_controller turtle_sensor
```

You should see the following output:

![Turtle sensor](/assets/images/ros/subscriber/color.png)

## Things to try

1. Add subscriber to the "turtle1/pose" topic and print out the turtle's pose.

---
[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/publisher.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/service-client.md %})
