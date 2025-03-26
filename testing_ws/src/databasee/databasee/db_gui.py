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

from PyQt5.QtWidgets import QApplication, QMainWindow, QCalendarWidget, QLabel, QVBoxLayout, QWidget, QMessageBox, QPushButton
from PyQt5.QtCore import QDate, Qt
from PyQt5.QtGui import QTextCharFormat

import cv2
from PyQt5.QtGui import QPixmap


class DB(Node):
    def __init__(self):
        super().__init__('database')
        # 주문 내역 구독
        self.sub = self.create_subscription(Ordermsg, 'order_topic', self.sub_callback, 10)
        conn = sqlite3.connect('/home/oh/project/drive1/sql_test/menu.db')
        cursor = conn.cursor()

                # 선택된 날짜에 맞는 table선택
        order = 'Select * from menu'

        cursor.execute(order)
        # 모든 내용 긁어오기
        result = cursor.fetchall()

        conn.commit()
        conn.close()

        ff = []
        mm = []
        for i in result:

            ff.append(i[0])
            mm.append(i[2])


        self.declare_parameter('food', ff)
        self.declare_parameter('money', mm)


        self.a = self.get_parameter('money')

        print(self.a.to_parameter_msg().value.integer_array_value[0])


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
        print(self.node.a)
        print('\n\n\n\n')
        # db에 연결
        conn = sqlite3.connect('/home/oh/project/drive1/testing_ws/src/databasee/databasee/orders.db')
        cursor = conn.cursor()

        # data.data는 2025_01_24 형태
        d = data.split(' ')

        # 1달치
        if len(d[1]) == 2:
            month = d[3]+'_0'+d[1][0]
        else:
            month = d[3]+'_'+d[1][:2]

        month_day = np.arange(1,32,1)
        month_whole_per_day = np.zeros(31)

        for i in range(1,32,1):
            try:
                if len(str(i)) == 2:
                    month = month + '_' + str(i)
                else:
                    month = month + '_0' + str(i)

                month_order = 'Select * from orders'+month
                cursor.execute(month_order)
                month_result = cursor.fetchall()

                conn.commit()

                for j in month_result:
                    month_whole_per_day[i-1] += int(j[3])

                month = month[:7]
            except:
                month = month[:7]
        
        month_whole = sum(month_whole_per_day)
        # 1주일치



        # 하루치

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
        # 종합 금액
        mo = []
        # 총합 갯수
        co = []
        # 총액
        whole = 0

        # 리스트에 없으면 append, 있으면 위치에 맞게 갯수 증가
        for i in result:
            if i[1] in fo:
                mo[fo.index(i[1])] += int(i[4])
                co[fo.index(i[1])] += int(i[2])
            else:
                fo.append(i[1])
                mo.append(int(i[4]))
                co.append(int(i[2]))

        for i in range(len(fo)):
            whole += mo[i]


        me = ''
        for i in range(len(fo)):
            me = me + fo[i] + ' :    ' + str(co[i]) + '개  -   ' + str(mo[i]) + '원\n'
        me = me + '\n가장 잘 팔린 메뉴 : ' + fo.index(max(co)) + '\n'
        me = me + '\n일간 총 판매액 : ' + str(whole) + '원\n\n'
        me = me + '월간 총 판매액 : ' + str(month_whole) + '원'

        

        # 시간대별로 나누기 위해서(영업시간 10:00:00 - 21:59:59, 2시간 단위)
        food = []
        count_by_time = []
        whole_by_time = [0, 0, 0, 0, 0, 0]
        ti = np.arange(10, 22, 2)

        for i in result:
            hour = int(i[0].split(':')[0])
            ind = hour//2-5

            whole_by_time[ind] += int(i[3])

            if i[1] in food:
                count_by_time[food.index(i[1])][ind] += int(i[3])
            else:
                food.append(i[1])
                count_by_time.append([0, 0, 0, 0, 0, 0])
                count_by_time[food.index(i[1])][ind] = int(i[3])



        # 그래프 그리기
        # 일별 매출
        for i in range(len(month_day)):
            plt.plot(month_day, month_whole_per_day)
        plt.grid()
        plt.title(f'{month} sales per day')
        plt.savefig('month.png')

        plt.cla()

        # 1일 시간별 총 매출
        plt.plot(ti, whole_by_time)
        plt.grid()
        plt.legend()
        plt.title(f'{a} sales per hour')
        #plt.show()
        
        plt.savefig('1day_whole.png')

        plt.cla()

        # 1일 시간별 메뉴 총 매출
        for i in range(len(food)):
            plt.plot(ti, count_by_time[i], label = food[i])
        plt.grid()
        plt.legend()
        plt.title(f'{a} sales per hour')
        #plt.show()
        
        plt.savefig('1day.png')

        plt.cla()

        #img = cv2.imread('savefig_default.png')
        #frame = cv2.imwrite('test.jpg',img)
        #frame_ = bridge.cv2_to_imgmsg(img, 'bgr8')
            
        #p.data = frame.tobytes()
        #self.pub_pic.publish(frame_)

        pixmap_m = QPixmap('month.png')
        lbl_img_m = QLabel()
        lbl_img_m.setPixmap(pixmap_m)
        lbl_img_m.show()

        pixmap = QPixmap('1day.png')

        lbl_img = QLabel()
        lbl_img.setPixmap(pixmap)
        lbl_img.show()

        # 팝업창 관련
        messagebox = QMessageBox()
        messagebox.setWindowTitle(f"{a} 판매액")
        messagebox.setText(f"{me}")

        ok_button = QPushButton("OK")
        messagebox.addButton(ok_button, QMessageBox.AcceptRole)
        messagebox.exec_()


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