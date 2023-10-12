import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist

from action_tutorials_interfaces.action import MessageTurtleCommands


class TurtleActionServer(Node):

    def __init__(self):
        super().__init__('turtle_action_server')
        self.odometr = 0
        self._cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Accepted goal cmd: {goal_handle.request.cmd}")

        cmd_vel_msg = Twist()
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = self.odometr

        cmd = goal_handle.request.cmd

        if (cmd == "forward"):
            cmd_vel_msg.linear.x = 1.
            for i in range(goal_handle.request.s):
                self._cmd_vel_pub.publish(cmd_vel_msg)
                self.odometr += 1
                feedback_msg.odom = self.odometr
                self._cmd_vel_pub.publish(cmd_vel_msg)
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)
        elif (cmd == "turn_right"):
            cmd_vel_msg.angular.z = -1.0
            self._cmd_vel_pub.publish(cmd_vel_msg)
            goal_handle.publish_feedback(feedback_msg)
        elif (cmd == "turn_left"):
            cmd_vel_msg.angular.z = 1.0
            self._cmd_vel_pub.publish(cmd_vel_msg)
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = TurtleActionServer()

    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()