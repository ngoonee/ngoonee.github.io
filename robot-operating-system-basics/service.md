---
title: Service
layout: default
parent: Robot Operating System (ROS) Basics
nav_order: 7
---

## Service

In this section, we will create a service node that will reset turtlesim and empty the simulation window. Create a new node name "turtle_service.py" in the "turtle_controller" package and key in the following code:

```python
# Import necessary modules
from turtlesim.srv import Kill
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node


# Define a class for the TurtleEmpty service, which is a subclass of rclpy's Node class
class TurtleEmpty(Node):
    def __init__(self):
        # Call the constructor of the base class (Node) with the node name "turtle_empty_service"
        super().__init__("turtle_empty_service")

        # Create a service for the "/turtle_empty" service of type Empty
        self.srv = self.create_service(Empty, "turtle_empty", self.turtle_empty)

        # Create clients to interact with the "/reset" and "/kill" services from turtlesim
        self.turtle_reset = self.create_client(Empty, "/reset")
        self.turtle_kill = self.create_client(Kill, "/kill")

        # Wait for the "/reset" and "/kill" services to become available, with a timeout of 1 second
        while not self.turtle_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        while not self.turtle_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # Log a message indicating that the TurtleEmpty service has started
        self.get_logger().info("Turtlesim empty service started.")

    # Define a callback method for the "turtle_empty" service
    def turtle_empty(self, request, response):
        # Log a message indicating that the turtlesim is being reset
        self.get_logger().info("resetting turtlesim")

        # Create a request object of the type "Empty.Request" to reset the turtlesim
        req = Empty.Request()

        # Call the "/reset" service asynchronously and attach a callback for the response
        self.future_reset = self.turtle_reset.call_async(req)
        self.future_reset.add_done_callback(self.reset_callback)

        # Return an empty response to the service client
        return Empty.Response()

    # Define a callback method for the "/reset" service response
    def reset_callback(self, future):
        # Log a message indicating that the turtlesim has been reset successfully
        self.get_logger().info("turtlesim reset success, killing turtle 1...")

        # Create a request object of the type "Kill.Request" to kill "turtle1"
        req_kill = Kill.Request()
        req_kill.name = "turtle1"

        # Call the "/kill" service asynchronously and attach a callback for the response
        self.future_kill = self.turtle_kill.call_async(req_kill)
        self.future_kill.add_done_callback(self.kill_callback)

    # Define a callback method for the "/kill" service response
    def kill_callback(self, future):
        # Log a message indicating that the turtlesim is now empty
        self.get_logger().info("turtlesim is empty")


# Define the main function
def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the TurtleEmpty class to provide the "turtle_empty_service"
    service = TurtleEmpty()

    # Spin the node, allowing it to handle service requests and callbacks
    rclpy.spin(service)

    # Clean up and shut down the ROS client library
    rclpy.shutdown()


# Entry point of the script, check if it's the main module and call the main function
if __name__ == "__main__":
    main()

```

Add a new entry point to the newly created file in "setup.py" for this code and build the package.

After starting the node, there will be a new service called "turtle_empty" that can be called to reset turtlesim and empty the simulation window. use the command below to call the service:

```bash
ros2 service call /turtle_empty std_srvs/srv/Empty {}
```

## Things to try

1. Spawn a new turtle named "turtle0" after emptying the simulation window.

---
[Previous]({{ site.baseurl }}{% link robot-operating-system-basics/service-client.md %}) | [Next]({{ site.baseurl }}{% link robot-operating-system-basics/action-client.md %})
