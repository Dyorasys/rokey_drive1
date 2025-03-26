# 실험용 db 만들기 위해서 데이터 입력

import sqlite3

conn = sqlite3.connect('/home/oh/project/drive1/sql_test/menu.db')
cursor = conn.cursor()

b = input('음식 (cheese / food / 5000 형식) : ').split(' / ')
food = b[0]
type = b[1]
money = b[2]

exi = 'create table if not exists menu (food TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, type TEXT, money INTEGER)'
order = 'insert into menu (food, type, money) values (?, ?, ?)'

cursor.execute(exi)
cursor.execute(order, (food, type, money))

conn.commit()
conn.close()