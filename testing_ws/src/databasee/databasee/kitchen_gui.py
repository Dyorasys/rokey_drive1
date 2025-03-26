# 2025.01.23 - 이름은 안뜨는데 체크한 거 없애는 기능

from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from datetime import datetime
import signal
import sys
import rclpy
import sqlite3
import threading
from functools import partial
import argparse


class NODE(Node):
    def __init__(self, args):
        super().__init__('kitchen')
        self.args = args
        self.table_num = int(self.args.table_num)
        self.emit_signal = None

        self.subscription_list = []
        for i in range(1,self.table_num +1):
            self.subscription_list.append(
                self.create_subscription(
                    String, 
                    f'/table{i}/message', 
                    partial(self.subscription_callback, id = i),
                    10
                    )
                )

    def subscription_callback(self, msg, id):
        message = msg.data
        message = f'table{id},' + msg.data
        self.get_logger().info(f'Received message: {message}')

        if self.emit_signal is not None:
            self.emit_signal(message)
        else:
            self.get_logger().info(f'Node-Gui no connected')

    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func

class GUI(QMainWindow):
    message_received = Signal(str)
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.args = self.node.args
        self.message_received.connect(self.received_message)

        self.price_list = {'KIMBAB' : 3000}
        self.setupUi()
        self.initialize_signal()
        self.connect_database()


    def setupUi(self):
        self.setObjectName(u'MainWindow')
        self.setWindowTitle('kitchen')
        self.resize(1200, 800)

        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName(u"centralwidget")

        self.vbox = QVBoxLayout(self.centralwidget)
        self.result = QGridLayout(self.centralwidget)

        # table 1
        self.groupBox1 = QGroupBox(self.centralwidget)
        self.groupBox1.setObjectName(u"groupBox1")

        self.label_table1 = QLabel(self.centralwidget)
        self.label_table1.setObjectName(u'label_table1')
        self.label_table1.setText(f'Table 1')

        self.checkBox1 = QPushButton(self.centralwidget)
        self.checkBox1.setObjectName(u"checkBox1")

        self.textBrowser_table_list1 = QTextBrowser(self.centralwidget)
        self.textBrowser_table_list1.setObjectName(u"listView")

        self.vbox1 = self.vbox

        self.vbox1.addWidget(self.label_table1)
        self.vbox1.addWidget(self.textBrowser_table_list1)
        self.vbox1.addWidget(self.checkBox1)

        self.groupBox1.setLayout(self.vbox1)
        '''
        # table 2
        self.groupBox2 = QGroupBox(self.centralwidget)
        self.groupBox2.setObjectName(u"groupBox2")

        self.label_table2 = QLabel(self.centralwidget)
        self.label_table2.setObjectName(u'label_table1')
        self.label_table2.setText(f'Table 1')

        self.checkBox2 = QPushButton(self.centralwidget)
        self.checkBox2.setObjectName(u"checkBox2")

        self.textBrowser_table_list2 = QTextBrowser(self.centralwidget)
        self.textBrowser_table_list2.setObjectName(u"listView")

        self.vbox2 = self.vbox

        self.vbox2.addWidget(self.label_table2)
        self.vbox2.addWidget(self.textBrowser_table_list2)
        self.vbox2.addWidget(self.checkBox2)

        self.groupBox1.setLayout(self.vbox1)

        ########
        '''
        # 서빙
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        #self.pushButton.setGeometry(QRect(550, 420, 161, 91))
        self.pushButton.clicked.connect(self.button_clicked_done)
        self.pushButton.setText('Done')


        ###########
        self.result.addWidget(self.groupBox1,(0,0))
        #self.result.addWidget(self.groupBox2,(0,1))
        self.result.addWidget(self.pushButton,(0,1))

        self.setLayout(self.result)

        #self.setCentralWidget(self.centralwidget)

        self.textBrowser_table_list = [self.textBrowser_table_list1]




    def received_message(self, message):
        message_list = message.split(',')
        table_id = int(message_list[0].replace('table',''))
        message_list = message_list[1:]

        for i in range(len(message_list)//2):
            manu = message_list[i*2]
            num  = message_list[i*2 +1]
            message = f'{manu}  x{num}'
            self.textBrowser_table_list[table_id -1].append(message)
            self.insert_data(table_id, manu, int(num))

    def button_clicked_complete1(self):
        message = '---complete---'
        self.textBrowser_table1.append(message)

    def button_clicked_done(self):
        if self.checkBox1.isChecked():
            self.textBrowser_table_list1.clear()

    def initialize_signal(self):
        self.node.set_emit_signal(self.message_received.emit)

    def connect_database(self):
        # 데이터 베이스 연결 및 생성성
        self.conn = sqlite3.connect('database.db')
        self.cursor = self.conn.cursor()
        # 데이터 베이스에 테이블 생성성
        self.cursor.execute('''
CREATE TABLE IF NOT EXISTS orders (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    table_number INTEGER NOT NULL,
    manu TEXT NOT NULL,
    num INTEGER NOT NULL,
    time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
)
''')
        self.conn.commit()

    # 데이터 추가가
    def insert_data(self, table_id, manu, num):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.cursor.execute("INSERT INTO orders (table_number, manu, num, time) VALUES (?, ?, ?, ?)", 
                    (table_id, manu, num, current_time))
        self.conn.commit()


    def button_clicked_showrevenue(self):
        # 데이터 베이스에서 데이터 참조조
        self.cursor.execute("SELECT manu, num FROM orders")
        rows = self.cursor.fetchall()
        sum_ = 0
        for manu, num in rows:
            sum_ += self.price_list[manu] *num
        
        # 팝업창 관련
        messagebox = QMessageBox()
        messagebox.setWindowTitle(" ")
        messagebox.setText(f"total revenue : {sum_}")
        ok_button = QPushButton("OK")
        messagebox.addButton(ok_button, QMessageBox.AcceptRole)
        messagebox.exec_()

# sys.argv : 런치 파일에서 받은 모든 명령어 인자 
def main(args=sys.argv):
    rclpy.init(args=args)
    # ros에서만 사용되는 명령어 인자 제거
    args = rclpy.utilities.remove_ros_args(args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-table_num', type=str, default='1', help='table num')
    if args[1:] is None:
        args = parser.parse_args()
    else:
        args = parser.parse_args(args[1:])

    node = NODE(args)
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