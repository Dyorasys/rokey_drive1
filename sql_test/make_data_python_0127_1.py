# 실험용 db 만들기 위해서 데이터 입력

import sqlite3

conn = sqlite3.connect('/home/oh/project/drive1/sql_test/orders.db')
cursor = conn.cursor()

date = input('2025_01_27 형식 : ')
a = input('시간 (12:01:94 형식) : ')
b = input('음식 (cheese / 3 형식) : ').split(' / ')
id = b[0]
count = b[1]
category = b[2]

exi = 'create table if not exists orders'+date+' (time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, menu TEXT, count INTEGER, category TEXT)'
order = 'insert into orders'+date+' (time, menu, count, category) values (?, ?, ?, ?)'

cursor.execute(exi)
cursor.execute(order, (a, id, count, category))

conn.commit()
conn.close()