import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class result(Node):
    def __init__(self):
        super().__init__('want_result')
        self.pub_ = self.create_publisher(String, 'static',10)
        self.t = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        a = '2025_01_27 / 2'
        msg = String()
        msg.data = a
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = result()

    rclpy.spin_once(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

