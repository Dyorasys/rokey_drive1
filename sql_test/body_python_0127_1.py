import sqlite3
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np

conn = sqlite3.connect('/home/oh/project/drive1/sql_test/t.db')
cursor = conn.cursor()

a = input('원하는 날짜 (2025_01_27 형태): ')
b = input('하루 매출 - 1, 시간별 매출 - 2 : ')

order = 'Select * from orders'+a
cursor.execute(order)
result = cursor.fetchall()
print(result)

conn.commit()
conn.close()

# 하루 매출이 보고 싶어
if b == '1':
    fo = []
    co = []
    whole = 0

    for i in result:
        if i[1] in fo:
            co[fo.index(i[1])] += int(i[2])
        else:
            fo.append(i[1])
            co.append(int(i[2]))

    for i in range(len(fo)):
        print(fo[i] + ' ' + str(co[i]) + '개 \n')

# 시간별 매출이 보고 싶어
# 10:00 - 22:00
elif b == '2':
    # 영업시간을 2시간씩 나눠서 보여줌
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
            print(food)
            count_by_time.append([0, 0, 0, 0, 0, 0])
            print(count_by_time)
            count_by_time[food.index(i[1])][ind] = int(i[2])

    # 그래프 그리기
    for i in range(len(food)):
        plt.plot(ti, count_by_time[i], label = food[i])
    plt.legend()
    plt.title(f'{a} sales per hour(2h)')

    plt.show()
    plt.savefig('savefig_default.png')