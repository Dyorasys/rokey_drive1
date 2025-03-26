import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys

class Testing(Node):
    def __init__(self):
        super().__init__('testing')
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.sub_callback, 10)

    def sub_callback(self, msg):
        print('---------------')
        print('msg')
        print(msg)
        print('type')
        print(type(msg))
        print('pose')
        print(msg.pose.pose.position.x)
'''
header:
  stamp:
    sec: 80
    nanosec: 619000000
  frame_id: map
pose:
  pose:
    position:
      x: 0.09245233311274756
      y: -0.34819794938510573
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.8790069278902065
      w: 0.4768089981544195
  covariance:
  - 0.03416880146585473
  - -0.011029525074440646
'''
def main(args=None):
    rclpy.init(args=args)

    node = Testing()

    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()