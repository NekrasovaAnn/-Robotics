import sys
import math as m

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveToGoal(Node):

    def __init__(self):
        super().__init__('move_to_goal')
        # Создаем узел
        # Стркутура куда идти
        self._cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
        # Структура где черепаха и куда смотрит
        self._pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 0)
        # Сначала черепаха в центре поля
        self.pose_data = Pose(x=5.54, y=5.54, theta=0.)  

    def __call__(self, x, y, theta):
        self._move_to_goal(x, y, theta)

    def pose_callback(self, msg):
        self.pose_data = msg

    # Где черепаха
    def _get_turtle_pos(self):
        return self.pose_data

    # Как сейчас будет двигаться черепаха
    def _send_turtle_msg(self, x_speed: float = 0., angle_speed: float = 0.):
        msg = Twist()
        # Линейная скорость
        msg.linear.x = x_speed
        # Угловая скорость
        msg.angular.z = angle_speed

        # Бежим!
        self._cmd_vel_pub.publish(msg)
        #time.sleep(0.5)

    def _move_to_goal(self, x, y, theta):
        # Где черепаха?
        start_position = self._get_turtle_pos()
        # Здесь черепаха!
        self.get_logger().info(f"Start position: {start_position.x} {start_position.y} {start_position.theta}")
        # Сюда идет черепаха
        self.get_logger().info(f"Goal: {x} {y} {theta}")

        x_d = x - start_position.x
        y_d = y - start_position.y

        # Черепаха считает как надо двигаться, чтобы добраться до цели
        dst_target = m.sqrt(x_d ** 2 + y_d ** 2)
        ang_target = m.atan2(y_d, x_d) - start_position.theta

        # Черепаха говорит, как она планирует добираться
        self.get_logger().info(f"\t{dst_target} {ang_target}")

        # Если черепаха еще не добралась до цели, то она двигается к ней
        if (abs(dst_target) > 0.1):
            if (abs(ang_target) > 0.1):
                self._send_turtle_msg(angle_speed=ang_target)
            self._send_turtle_msg(dst_target)

        # Проверяем, надо ли черепахе поворачиваться
        ang_target = theta - ang_target

        if (abs(ang_target) > 0.1):
            self._send_turtle_msg(angle_speed=ang_target)

        # Черепаха пришла!
        self.get_logger().info("Goal Reached")
        quit()


def main():
    rclpy.init()

    # Создаем узел
    move_to_goal = MoveToGoal()
    # Передаем черепахе куда идти
    move_to_goal(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    # Заставляем черепаху двигаться
    rclpy.spin(move_to_goal)

    # Уничтожаем то, что создавали
    move_to_goal.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()