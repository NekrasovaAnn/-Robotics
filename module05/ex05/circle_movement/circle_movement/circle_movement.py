import rclpy
from rclpy.node import Node
import sys
from math import pi
from geometry_msgs.msg import Twist
import time

class Circle(Node):

    def __init__(self):
        super().__init__('circle')
        self.x = True
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(3, self.move_to_goal)
        self.radius = self.declare_parameter('radius', 1.0).get_parameter_value().double_value

    def move_to_goal(self):
        if (self.x):
            radius = self.radius
            tw = Twist()
            tw.linear.x = 1.0
            # tw.angular.z = radius * 2 * pi
            self.publisher.publish(tw)
            self.x = False
        else:
            tw = Twist()
            tw.linear.x = 0.0
            self.publisher.publish(tw)
            self.x = True


def main():
    rclpy.init()
    circle= Circle()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()