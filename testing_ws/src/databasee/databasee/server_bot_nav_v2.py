import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget
import queue
import time

class TurtleBot3Navigation(Node):
    def __init__(self):
        super().__init__('turtlebot3_navigation')

        self.delivered = True
        self.table_queue = []
        self.current_goal = None

        self.queue = queue.Queue()

        self.goal_position = {
            0: self.create_pose_stamped(
                                        x=-0.361966341733935,
                                        y=0.5080662965774536
            ),
            1: self.create_pose_stamped(
                                        x=0.914523534545353,
                                        y=1.075345353542435
            ),
            2: self.create_pose_stamped(
                                        x=0.932512434124353,
                                        y=-0.028251242353524
            ),
            3: self.create_pose_stamped(
                                        x=0.9825142423543254,
                                        y=-1.155142343541242
            ),
            4: self.create_pose_stamped(
                                        x=2.0253779888153076,
                                        y=1.0842508745193481
            ),
            5: self.create_pose_stamped(
                                        x=2.020387806892395,
                                        y=-0.003149221162796
            ),
            6: self.create_pose_stamped(
                                        x=1.970758409500122,
                                        y=-1.1505086946487427
            ),
            7: self.create_pose_stamped(
                                        x=3.116918592453003,
                                        y=1.0673075485229492
            ),
            8: self.create_pose_stamped(
                                        x=3.1166799068450928,
                                        y=0.139125794172287
            ),
            9: self.create_pose_stamped(
                                        x=3.1468627548217773,
                                        y=-1.1403740763664246
            ),
        }


        self.create_subscription(Int32MultiArray, '/run', self.run_callback, QoSProfile(depth=10))

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.app = QApplication([])
        self.window = QWidget()
        self.push_button = QPushButton('수령', self.window)
        self.push_button.clicked.connect(self.handle_push_button)
        self.push_button.setEnabled(False)
        self.window.show()

    def run_callback(self, msg):
        if self.delivered:
            for i in range(len(msg.data)):
                self.queue.put(msg.data[i])
            print(self.queue.queue)
            time.sleep(2)
            self.table_queue = self.queue
            self.delivered = False
            self.execute_navigation(self.table_queue.queue[0])

    def handle_push_button(self):
        if self.table_queue:
            print(self.queue.queue)
            self.table_queue.get()
            next_goal = self.queue.queue[0] 
            time.sleep(2)
            self.execute_navigation(next_goal)
            self.push_button.setEnabled(False)
        if not self.table_queue:
            self.delivered = True

    def create_pose_stamped(self, x, y, z=0.0):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def execute_navigation(self, goal_point=None):
        #Action Debug
        wait_count = 1
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                self.set_emit_signal(['append_textBrowser', message])
                self.get_logger().error("네비게이션 액션 서버 무응답")
                return False
            wait_count += 1        
            
        if goal_point not in self.goal_position:
            self.get_logger().error(f"없는 테이블입니다. : {goal_point}")
            return
        
        if not self.table_queue or self.table_queue.queue[0]==0:
            self.get_logger().error("서비스 대기 중입니다.")
            self.delivered = True
            return          
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose = self.goal_position[goal_point]

        self.get_logger().info(f"{goal_point}번 테이블로 이동 시작")
        
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_to_pose_client_feedback)
        self.send_goal_future.add_done_callback(self.nav_to_pose_client_action_goal)
        
        return True
    
    
    def nav_to_pose_client_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = "[WARN] Action goal rejected."
            self.set_emit_signal(['append_textBrowser', message])
            return

        message = "[INFO] Action goal accepted."
        self.set_emit_signal(['append_textBrowser', message])
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.nav_to_pose_client_result)

    
    def nav_to_pose_client_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        self.get_logger().info("Action feedback: {0}".format(action_feedback))
    

    def nav_to_pose_client_result(self, future):
        result = future.result()
        if result.status == 4:
            message = "[INFO] Action succeeded!."
            self.set_emit_signal(['append_textBrowser', message])
            
            self.get_logger().info(f"{self.current_goal}번 테이블, 주문하신 음식입니다.")
            self.push_button.setEnabled(True)
            
        elif result.status == 2:  # CANCELED 상태
            self.get_logger().error("목표가 취소되었습니다.")
        elif result.status == 3:  # ABORTED 상태
            self.get_logger().error("목표에 도달하지 못했습니다.")
        else:
            self.get_logger().error("알 수 없는 상태입니다.")

    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func


    '''
    def execute_navigation(self, goal_point=None):
        if not self.table_queue:
            self.get_logger().error("서비스 대기 중입니다.")
            self.delivered = True
            return

        goal_point = self.table_queue[0]

        if goal_point not in self.goal_position:
            self.get_logger().error(f"없는 테이블입니다. : {goal_point}")
            return

        self.current_goal = goal_point
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_position[goal_point]

        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info(f"{goal_point}번 테이블로 이동 시작")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("목표가 거부되었습니다.")
                return

            self.get_logger().info("목표가 수락되었습니다.")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def result_callback(self, future):
        result = future.result()
    
        # ActionResult의 상태 코드를 확인
        if result.status == 3:  # SUCCEEDED 상태
            self.get_logger().info(f"{self.current_goal}번 테이블, 주문하신 음식입니다.")
            self.push_button.setEnabled(True)
        elif result.status == 2:  # CANCELED 상태
            self.get_logger().error("목표가 취소되었습니다.")
        elif result.status == 4:  # ABORTED 상태
            self.get_logger().error("목표에 도달하지 못했습니다.")
        else:
            self.get_logger().error("알 수 없는 상태입니다.")

    '''
    
def main(args=None):
    rclpy.init(args=args)
    navigation_node = TurtleBot3Navigation()

    ros_thread = threading.Thread(target=rclpy.spin, args=(navigation_node,))
    ros_thread.start()

    try:
        navigation_node.app.exec_()
    finally:
        navigation_node.destroy_node()
        ros_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
