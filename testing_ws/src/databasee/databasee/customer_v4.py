import sys
import threading
import queue
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory

import os

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from std_msgs.msg import String
from functools import partial
import argparse

class NODE(Node):
    def __init__(self):
        super().__init__('node')

        qos_profile = QoSProfile(depth=5)
        self.message_publisher = self.create_publisher(String, 'message', qos_profile)

        self.queue = queue.Queue()
        self.timer = self.create_timer(0.1, self.publish_message)

    def publish_message(self):
        while not self.queue.empty():
            message = self.queue.get()
            msg = String()
            msg.data = message
            self.message_publisher.publish(msg)
            self.get_logger().info(f'Published message: {message}')


class GUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.package_name = 'testing'
        self.pkg_path = os.path.join(get_package_share_directory(self.package_name))

        self.node = node
        self.order_list = {}
        self.price_list = {'KIMBAB' : 3000}
        self.total = 0

        self.setupUi()

    def setupUi(self):
        self.setObjectName(u"MainWindow")
        self.setWindowTitle(f"{self.node.get_namespace()[1:]}")
        self.resize(687, 351)

        self.centralwidget = QWidget(self)

        self.pushButton_manu1 = QPushButton(self.centralwidget)
        self.pushButton_manu1.setObjectName(u"manu1")
        self.pushButton_manu1.setGeometry(QRect(20, 180, 181, 31))
        self.pushButton_manu1.setText(f"KIMBAB {self.price_list['KIMBAB']}₩")
        self.pushButton_manu1.clicked.connect(partial(self.button_clicked_manu, 'KIMBAB'))

        self.label_image1 = QLabel(self.centralwidget)
        self.label_image1.setObjectName(u"image1")
        self.label_image1.setGeometry(QRect(20, 70, 181, 91))
        self.load_manuimage(self.label_image1 ,'KIMBAB.jpg')

        self.textBrowser_orderlist = QTextBrowser(self.centralwidget)
        self.textBrowser_orderlist.setObjectName(u"order")
        self.textBrowser_orderlist.setGeometry(QRect(480, 90, 161, 141))

        self.label_total = QLabel(self.centralwidget)
        self.label_total.setObjectName(u"total")
        self.label_total.setGeometry(QRect(480, 250, 161, 31))
        self.label_total.setText(f"total : {self.total}₩")

        self.pushButton_reset = QPushButton(self.centralwidget)
        self.pushButton_reset.setObjectName(u"reset")
        self.pushButton_reset.setGeometry(QRect(480, 300, 71, 23))
        self.pushButton_reset.setText("reset")
        self.pushButton_reset.clicked.connect(self.button_clicked_reset)

        self.pushButton_pay = QPushButton(self.centralwidget)
        self.pushButton_pay.setObjectName(u"pay")
        self.pushButton_pay.setGeometry(QRect(570, 300, 71, 23))
        self.pushButton_pay.setText("pay")
        self.pushButton_pay.clicked.connect(self.button_clicked_pay)
        
        self.setCentralWidget(self.centralwidget)

    # 라벨에 이미지 삽입
    def load_manuimage(self, label, image_name):
        image_path = os.path.join(self.pkg_path, 'manu_images', image_name)
        pixmap = QPixmap(image_path)
        label.setPixmap(pixmap)
        label.setScaledContents(True)


    def button_clicked_manu(self, manu):
        if not manu in self.order_list:
            self.order_list[manu] = 1
        else:
            self.order_list[manu] += 1
        self.total += self.price_list[manu]

        self.label_total.setText(f"total : {self.total}₩")

        self.textBrowser_orderlist.clear()
        for key,val in self.order_list.items():
            self.textBrowser_orderlist.append(f'{key} x{val}')

    def button_clicked_reset(self):
        self.total = 0
        self.label_total.setText(f"total : {self.total}₩")

        self.order_list = {}
        
        self.textBrowser_orderlist.clear()
    
    def button_clicked_pay(self):
        message = ''
        for key,val in self.order_list.items():
            message += f'{key},{val},'
        message=message[:-1]
        self.node.queue.put(message)
        self.button_clicked_reset()

        
def main(args=sys.argv):
    # 네임 스페이스, 파라미터 등의 인자 값 적용용
    rclpy.init(args=args)

    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()
    
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        sys.exit(0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()