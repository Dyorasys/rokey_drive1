import pymysql

# 날짜별로 테이블 생성
# 날짜별로 확인 가능
# 날짜 테이블 생성시 - 쓰지 말 것

# MySQL 데이터 베이스 연결
# !!!! 보안문제 !!!!
# ID, PW를 받아와서 여기에 입력???

conn = pymysql.connect(host = '127.0.0.1', user = 'tester2', password = 'Wpdntm1211!',
                     port = 3306, db = 'chart', charset='utf8')

a = input('0 : order, number : 통계')
if a == '0':

    t = input('time :')
    id = input('id : ')
    count = input('count : ')

    # 데이터 입력
    try:
        with conn.cursor() as cursor:
            # 오늘 매출표가 존재하지 않으면 생성
            exi = 'create table if not exists orders'+t+'(time char(20), id char(20), count char(20))'
            order = 'insert into orders'+t+' values (%s, %s, %s)'

            print(exi)
            # 내부 변수 바꾸기
            # order = 'update chart set major_nm = %s where professor_nm = %s'

            cursor.execute(exi)
            cursor.execute(order, (t, id, count))

        # 마지막에 명령을 실제 수행 
        print('write')
        conn.commit()
    except:
        print('error')
    finally:
        conn.close()

else:
    t = input('time : ')
    # 데이터 확인
    try:
        with conn.cursor() as cursor:
            order = 'Select * from orders'+t

            cursor.execute(order)
            result = cursor.fetchall()

            conn.commit()

            for i in result:
                print(i)
    finally:
        conn.close()

# DB연결 전에 없으면 생성?



