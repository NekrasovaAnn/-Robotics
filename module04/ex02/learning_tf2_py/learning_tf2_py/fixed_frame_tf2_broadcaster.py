from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('fixed_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter('radius', 3)
        self.declare_parameter('direction_of_rotation', 1)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        # Создаем новое преобразование
        t = TransformStamped()

        radius = self.get_parameter('radius').get_parameter_value().integer_value
        direction_of_rotation = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value

        if(abs(direction_of_rotation)!=1):
            self.get_logger().info('Error rotate, you must write 1 or -1')
            quit()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = 0.0
        # Смещение на заданное количество метров по y
        t.transform.translation.y = float(radius)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        # Направление вращения черепахи
        t.transform.rotation.w = float(direction_of_rotation)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
