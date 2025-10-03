---
title: Service Client
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 6
---

## Service

Service in ROS 2 is a mechanism for requesting and receiving data from a node. The node that provides the service is called the service server, while the node that requests the service is called the service client. The service server provides a service that can be requested by the service client. The service client requests the service from the service server and receives a response from the service server. The service server can also send a response to the service client.

## Service Client

In this section, we will create a service client node that spawn new turtle in turtle sim. The node, named "turtle_service_client", will be part of the project we created in the previous section. Create the file "turtle_service_client.py" in the "turtle_controller" package and key in the following code:

```python
# Import necessary modules
from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node

# Define a class for the TurtleColorClient, which is a subclass of rclpy's Node class
class TurtleColorClient(Node):
    def __init__(self):
        # Call the constructor of the base class (Node) with the node name "turtle_color_client"
        super().__init__("turtle_color_client")

        # Create a client to interact with the "/spawn" service from turtlesim
        self.cli = self.create_client(Spawn, "/spawn")

        # Wait for the service to become available, with a timeout of 1 second
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    # Define a method to send a request to the service to spawn a turtle
    def send_request(self, x, y, theta):
        # Create a request object of the type "Spawn.Request"
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta

        # Call the service asynchronously and store the future object for the response
        self.future = self.cli.call_async(req)

        # Spin the node until the future is complete (response received or timeout)
        rclpy.spin_until_future_complete(self, self.future)

        # Return the result of the future (response from the service)
        return self.future.result()

# Define the main function
def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the TurtleColorClient
    color_client = TurtleColorClient()

    # Send a request to spawn a turtle at position (1.5, 1.0) with an orientation of 0.0
    response = color_client.send_request(1.5, 1.0, 0.0)

    # Print the name of the spawned turtle obtained from the service response
    color_client.get_logger().info(f"Turtle spawned: {response.name}")

    # Clean up and shut down the ROS client library
    color_client.destroy_node()
    rclpy.shutdown()

# Entry point of the script, check if it's the main module and call the main function
if __name__ == "__main__":
    main()

```

Add a new entry point to the newly created file in "setup.py" for this code and build the package.

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller --symlink-install
```

Add a new entry point to the newly created file in "setup.py" for this code and build the package.

After running the node, you should see a new turtle spawned in the turtle sim window. You can control the turtle using the arrow keys using the command below. Remapping is used to change the topic name of the teleop turtle node to the newly spawned turtle.

```bash
# Remap teleop turtle to turtlename
ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/{turtlename}/cmd_vel
```

![Turtle spawn](/assets/images/ros/service-client/spawn.png)

## Things to try

1. Spawn multiple turtle at different location and orientation with 1 service client node
2. Control the different turtle using the arrow keys
3. Change the pen color of newly spawned turtle

---
[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/subscriber.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/service.md %})
