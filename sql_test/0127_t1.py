import sqlite3
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from food_msg.msg import Ordermsg

# 같은 디렉토리에 t.db 있어야 함
# 0을 누르면 주문, 갯수를 입력 가능 (str, int)
# 2025_01_24 형식으로 테이블이 저장
# 0을 누르지 않고 2025_01_24 누르면 통계 간단히 나옴
# ros와 연동시 sub, pub 자료형만 맞추기
# 그래프화, ros 연동 시험 중
# message를 'cheese 1'이런식으로 하나로 받아서 분할하는 형식으로 테스트
# 한번에 여러 음식이 들어오는건 구현중

class DB(Node):
    def __init__(self):
        super().__init__('database')
        # 주문 내역 구독
        self.sub = self.create_subscription(Ordermsg, 'food', self.sub_callback, 10)

        # 통계를 원할때
        self.pub = self.create_publisher(String, 'result', 10)
        # 통계 원하면 통계 내역 발행
        self.sub_k = self.create_subscription(String, 'static', self.sub_k_callback, 10)
        

    def sub_callback(self, data):
        # if 구문은 test할때 계속 토픽을 발행해서 추가. 추후 실제로 연결하면 if문 제거
        if data:

            # db에 연결
            conn = sqlite3.connect('t.db')
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


    # 통계를 원한다면
    def sub_k_callback(self, data):
        # db에 연결
        conn = sqlite3.connect('t.db')
        cursor = conn.cursor()

        # data.data는 2025_01_24 형태
        a = data.data
        order = 'Select * from orders'+a

        cursor.execute(order)
        # 모든 내용 긁어오기
        result = cursor.fetchall()

        conn.commit()
        conn.close()

        # 품목 이름
        re = []
        # 품목별 총 판매갯수
        co = []

        for i in result:
            if i[1] in re:
                co[re.index(i[1])] += i[2]
            else:
                re.append(i[1])
                co.append(i[2])

        # GUI에 보여주기 위한 데이터를 발행. 현재는 String으로만 제작. 추후 수정
        msg = String()
        a = ''
        for i in range(len(re)):
            a  = a + re[i] + str(co[i]) + '개 \n'
        msg.data = a

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DB()

    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()