import sqlite3
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np
import sys

import rclpy 
from rclpy.node import Node
from food_msg.msg import Ordermsg
from PyQt5.QtWidgets import QApplication, QWidget
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QCalendarWidget, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QDate, Qt
from PyQt5.QtGui import QTextCharFormat


class DB(Node):
    def __init__(self):
        super().__init__('database')
        # 주문 내역 구독
        self.sub = self.create_subscription(Ordermsg, 'order_topic', self.sub_callback, 10)

    # db에 정보 입력. 이 부분을 다른 코드로 옮기는게 더 좋을수도
    def sub_callback(self, data):
        # if 구문은 test할때 계속 토픽을 발행해서 추가. 추후 실제로 연결하면 if문 제거
        if data:

            # db에 연결
            conn = sqlite3.connect('/home/oh/project/drive1/testing_ws/src/databasee/databasee/t.db')
            cursor = conn.cursor()

            # 현재 시간
            current_time = datetime.now()
            date = current_time.strftime('%Y_%m_%d')
            time = current_time.strftime('%H:%M:%S')

            # 주문 내역
            for i in range(len(data.food_id)):
                id = data.food_id[i]
                count = data.food_count[i]

                # 데이터 입력
            
                # 오늘 매출표가 존재하지 않으면 생성. table 이름은 'odres2025_01_24' 형태로 뒤에 날짜가 붙음
                exi = 'create table if not exists orders'+date+'(time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, menu TEXT, count INTEGER)'
                order = 'insert into orders'+date+'(time, menu, count) values (?, ?, ?)'
            
            # table생성, 주문을 시간, 종류, 갯수 순으로 삽입
                cursor.execute(exi)
                cursor.execute(order, (time, id, count))

            # 실행
                conn.commit()
            # 연결 종료
            conn.close()
        
        else:
            pass

# 달력 보여주고, 날짜를 클릭하면 그에 대한 통계가 나옴.
class DBGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

    def initUI(self):
        self.setWindowTitle("QCalendarWidget Example")
        self.setGeometry(100, 100, 400, 300)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.calendar = QCalendarWidget(self)
        self.calendar.setGridVisible(True)
        self.calendar.setNavigationBarVisible(True)
        self.calendar.setFirstDayOfWeek(Qt.Monday)
        self.calendar.setSelectionMode(QCalendarWidget.SingleSelection)
        self.calendar.setMinimumDate(QDate(2000, 1, 1))
        self.calendar.setMaximumDate(QDate(2100, 12, 31))
        self.calendar.setHorizontalHeaderFormat(QCalendarWidget.SingleLetterDayNames)
        self.calendar.setVerticalHeaderFormat(QCalendarWidget.ISOWeekNumbers)
        
        self.calendar.clicked.connect(self.on_date_clicked)
        self.layout.addWidget(self.calendar)

        self.label = QLabel("Selected Date: None", self)
        self.layout.addWidget(self.label)

        # 특정 날짜 강조
        self.highlight_date(QDate(2023, 12, 25), Qt.red)

        self.show()

    # 클릭하면 통계 보여주기
    def on_date_clicked(self, date):
        self.label.setText(f"Selected Date: {date.toString()}")
        data = date.toString()
        self.sub_k_callback(data)

    # 클릭한 날짜 색깔 다르게
    def highlight_date(self, date, color):
        format = QTextCharFormat()
        format.setForeground(color)
        self.calendar.setDateTextFormat(date, format)

    #실제 통계 처리 되는 부분
    def sub_k_callback(self, data):
        # db에 연결
        conn = sqlite3.connect('/home/oh/project/drive1/testing_ws/src/databasee/databasee/t.db')
        cursor = conn.cursor()


        # 1달치
        


        # 1주일치

        # 하루치
        # data.data는 2025_01_24 형태
        d = data.split(' ')
        if len(d[1]) == 2:
            a = d[3]+'_0'+d[1][0]+'_'+d[2]
        else:
            a = d[3]+'_'+d[1][:2]+'_'+d[2]

        # 선택된 날짜에 맞는 table선택
        order = 'Select * from orders'+a

        cursor.execute(order)
        # 모든 내용 긁어오기
        result = cursor.fetchall()

        conn.commit()
        conn.close()

        # 음식 리스트
        fo = []
        # 종합 갯수
        co = []
        # 총액
        whole = 0

        # 리스트에 없으면 append, 있으면 위치에 맞게 갯수 증가
        for i in result:
            if i[1] in fo:
                co[fo.index(i[1])] += int(i[2])
            else:
                fo.append(i[1])
                co.append(int(i[2]))

        for i in range(len(fo)):
            print(fo[i] + ' ' + str(co[i]) + '개 \n')

        # 시간대별로 나누기 위해서(영업시간 10:00:00 - 21:59:59, 2시간 단위)
        food = []
        count_by_time = []
        ti = np.arange(10, 22, 2)

        for i in result:
            hour = int(i[0].split(':')[0])
            ind = hour//2-5

            if i[1] in food:
                count_by_time[food.index(i[1])][ind] += int(i[2])
            else:
                food.append(i[1])
                count_by_time.append([0, 0, 0, 0, 0, 0])
                count_by_time[food.index(i[1])][ind] = int(i[2])

        # 그래프 그리기
        for i in range(len(food)):
            plt.plot(ti, count_by_time[i], label = food[i])
        plt.grid()
        plt.legend()
        plt.title(f'{a} sales per hour')
        plt.show()
        ''''
        plt.savefig('savefig_default.png')

        bridge = CvBridge() 

        img = cv2.imread('savefig_default.png')

        #frame = cv2.imwrite('test.jpg',img)
        frame_ = bridge.cv2_to_imgmsg(img, 'bgr8')
            
        #p.data = frame.tobytes()
        self.pub_pic.publish(frame_)
        '''
            

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화

    app = QApplication(sys.argv)  # PyQt 애플리케이션 생성

    # GUI와 ROS2 구독자 노드 연결
    gui = DBGUI(None)  # GUI 생성
    subscriber_node = DB()  # ROS2 구독자 노드 생성
    gui.node = subscriber_node  # GUI에 노드 연결

    # ROS2 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(subscriber_node,))  # ROS2 스레드 생성 및 실행
    ros_thread.start()

    # GUI 실행
    gui.show()  # GUI 표시
    app.exec_()  # PyQt 이벤트 루프 실행

    # 종료
    subscriber_node.destroy_node()  # 노드 파괴
    rclpy.shutdown()  # ROS2 종료
    ros_thread.join()  # 스레드 종료 대기


if __name__ == '__main__':
    main()  # 메인 함수 실행