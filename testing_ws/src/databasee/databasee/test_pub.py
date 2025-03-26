import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile

class PublishRunCommand(Node):
    def __init__(self):
        super().__init__('publish_run_command')
        
        # /run 토픽에 퍼블리셔 생성
        self.publisher = self.create_publisher(Int32MultiArray, '/run', QoSProfile(depth=10))

        # 메시지 발행 주기 설정 (예: 1초마다)
        self.timer = self.create_timer(1.0, self.publish_run_message)

    def publish_run_message(self):
        # Int32MultiArray 메시지 생성
        msg = Int32MultiArray()
        
        # 메시지에 데이터를 추가
        msg.data = [1, 2, 3, 0]  # 예시로 1부터 5까지의 값을 배열에 넣음
        
        # /run 토픽으로 메시지 발행
        self.publisher.publish(msg)
        self.get_logger().info(f'발행된 메시지: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publish_node = PublishRunCommand()
    rclpy.spin_once(publish_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
