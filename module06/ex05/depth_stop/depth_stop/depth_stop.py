import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
import cv2
    
class Turtle(Node):
    def __init__(self):
        super().__init__('depth_stopping_Node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Image, '/depth/image', self.pose_callback, 1)
        self.timer = self.create_timer(0.2, self.depth_move)
        self.msg = Image()

    def pose_callback(self, data):
        self.msg = data

    def depth_move(self):
        message = Twist()
        image = self.msg.data
        if self.msg.width!=0:
            center_color = image[int(self.msg.width*self.msg.height/2+self.msg.width/2)]
            self.get_logger().info('dist {0} '.format(center_color))
       
            if(center_color != 0 and center_color != 127 and center_color !=128):
                message.linear.x = 0.0
            else:
                message.linear.x = 0.5
            message.angular.z = 0.0
            self.publisher_.publish(message)    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Turtle()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()