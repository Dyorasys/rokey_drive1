import rclpy
from rclpy.node import Node
from food_msg.msg import Ordermsg

class orders(Node):
    def __init__(self):
        super().__init__('from_kitchen')
        self.pub_ = self.create_publisher(Ordermsg, 'order_topic',10)
        self.t = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        a = input('음식 종류 / 음식 갯수 형태 . : ')
        msg = Ordermsg()
        li = a.split(' . ')
        fo = []
        co = []
        for i in range(len(li)):
            b = li[i].split(' / ')
            fo.append(b[0])
            co.append(int(b[1]))
        msg.food_id = fo
        msg.food_count = co
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = orders()

    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

