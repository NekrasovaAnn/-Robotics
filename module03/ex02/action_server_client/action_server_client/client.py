import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist

from action_tutorials_interfaces.action import MessageTurtleCommands


class TurtleActionClient(Node):

    def __init__(self):
        super().__init__("turtle_action_client")
        self._action_client = ActionClient(
            self, 
            MessageTurtleCommands, 
            'MessageTurtleCommands')
        self.goal_future = None
        self.result_future = None
        self.working = False

    def send_goal(self, cmd, s=0, angle=0):
        goal_msg = MessageTurtleCommands.Goal()

        goal_msg.cmd = cmd
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()

        self.goal_future = self._action_client.send_goal_async(goal_msg)
        self.goal_future.add_done_callback(self.goal_callback)
        self.working = True

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.result}")
        self.working = False


def main():
    rclpy.init()

    action_client = TurtleActionClient()

    action_client.send_goal("forward", 2)
    time.sleep(4)
    action_client.send_goal("turn_right", angle=90)
    time.sleep(4)
    action_client.send_goal("forward", 1)
    time.sleep(4)

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()