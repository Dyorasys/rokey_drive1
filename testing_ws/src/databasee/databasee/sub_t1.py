import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class ss(Node):
    def __init__(self):
        super().__init__('show_result')
        self.sub_ = self.create_subscription(Image, 'image',self.sub_,10)

    def sub_(self, data):
        frame_ = bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow('result', frame_)
        cv2.waitKey(0)



def main(args=None):
    rclpy.init(args=args)
    node = ss()

    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

