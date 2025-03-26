import sys

from PyQt5.QtWidgets import (

QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QListWidget, QMessageBox, QSpinBox, QListWidgetItem

)

import openai

import pyaudio

import wave

import speech_recognition as sr

import numpy as np

import time

import pyttsx3  # pyttsx3 라이브러리 임포트

from PyQt5.QtCore import QThread, pyqtSignal

# OpenAI API 키 설정

key = ""

client = openai.OpenAI(api_key=key)

# TTS 엔진 초기화

tts_engine = pyttsx3.init(driverName='espeak')  # espeak 엔진 사용

tts_engine.setProperty('rate', 140)  # 음성 속도 조정

tts_engine.setProperty('volume', 0.9)  # 볼륨 최대 설정

tts_engine.setProperty('voice', 'ko-KR')

# 녹음 설정

FORMAT = pyaudio.paInt16

CHANNELS = 1

RATE = 16000               # 샘플링 속도

FRAMES_PER_BUFFER = 2048   # 버퍼 크기

SILENCE_DURATION = 1  # 초기 침묵 감지 시간은 1초

# PyAudio 객체 초기화

p = pyaudio.PyAudio()

# 음성 인식 실패 횟수를 추적하는 변수

failed_recognition_count = 0

def save_audio(frames, filename):

wf = wave.open(filename, 'wb')

wf.setnchannels(CHANNELS)

wf.setsampwidth(p.get_sample_size(FORMAT))

wf.setframerate(RATE)

wf.writeframes(b''.join(frames))

wf.close()

recognizer = sr.Recognizer()

# 음성 파일을 텍스트로 변환 후 OpenAI API와 대화

history_messages = [

{"role": "system", "content": "당신의 재치있는 알바생이야. 항상 말을 재미있게 하지"}

]

def recognize_audio(filename):

with sr.AudioFile(filename) as source:

audio = recognizer.record(source)

try:

text = recognizer.recognize_google(audio, language="ko-KR")

return text

except sr.UnknownValueError:

return None

except sr.RequestError:

return None

def record_audio(silence_duration):

stream = p.open(format=FORMAT,

channels=CHANNELS,

rate=RATE,

input=True,

frames_per_buffer=FRAMES_PER_BUFFER)

frames = []

silence_start = None  # 침묵 시작 시간

while True:

data = stream.read(FRAMES_PER_BUFFER)

frames.append(data)

# 데이터가 16비트 정수로 변환

audio_data = np.frombuffer(data, dtype=np.int16)

# 평균 볼륨 계산 (신호 강도)

volume = np.abs(audio_data).mean()

if volume < 500:  # 볼륨이 낮은 경우 침묵으로 간주

if silence_start is None:

silence_start = time.time()  # 침묵 시작 시간 기록

else:

silence_start = None  # 음성이 감지되면 침묵 시작 시간 초기화

# 침묵 시간이 설정된 시간(silence_duration) 이상이면 종료

if silence_start is not None and (time.time() - silence_start) > silence_duration:

break

# 스트림 종료

stream.stop_stream()

stream.close()

return frames

waiting_for_response = False

class RecordAudioWorker(QThread):

recording_done = pyqtSignal(list)  # 녹음 결과를 전달할 신호

def __init__(self, silence_duration):

super().__init__()

self.silence_duration = silence_duration

def run(self):

frames = record_audio(self.silence_duration)

self.recording_done.emit(frames)  # 녹음 결과 전달

class TTSWorker(QThread):

def __init__(self, text):

super().__init__()

self.text = text

def run(self):

tts_engine.say(self.text)

tts_engine.runAndWait()

# GPTWorker 클래스 정의

class GPTWorker(QThread):

response_ready = pyqtSignal(str)  # 응답 신호

error_occurred = pyqtSignal(str)  # 오류 신호

def __init__(self, audio_file, history_messages, client):

super().__init__()

self.audio_file = audio_file

self.history_messages = history_messages

self.client = client

def run(self):

try:

# 음성을 텍스트로 변환

text = recognize_audio(self.audio_file)

if not text:

self.error_occurred.emit("음성을 인식할 수 없습니다.")

return

# OpenAI API 호출

self.history_messages.append({"role": "user", "content": text})

response = self.client.chat.completions.create(

model="gpt-3.5-turbo",

messages=self.history_messages,

temperature=0.2

)

answer = response.choices[0].message.content

self.response_ready.emit(answer)  # 응답 신호 방출

self.history_messages.append({"role": "assistant", "content": answer})

except Exception as e:

self.error_occurred.emit(f"GPT 호출 중 문제가 발생했습니다: {str(e)}")

class BurgerKiosk(QWidget):

def __init__(self):

super().__init__()

self.setWindowTitle("Burger Kiosk")  # 윈도우 제목 설정

self.setGeometry(100, 100, 600, 400)  # 창 크기와 위치 설정

# Layouts

self.main_layout = QVBoxLayout()  # 전체 레이아웃 (수직 배치)

self.menu_layout = QHBoxLayout()  # 메뉴 버튼들을 위한 가로 레이아웃

#self.order_layout = QVBoxLayout()  # 주문 내역을 위한 세로 레이아웃

# Menu items

# 메뉴에 표시될 항목 정의

self.burgers = ["Cheeseburger", "Chicken Burger", "Veggie Burger"]

self.burgers_mon = [5000, 6000, 7000]

self.side = ["Fries", "Onion Rings", "Chicken Nuggets"]

self.side_mon = [2000, 3000, 4000]

self.drinks = ["Coke", "Sprite", "Water"]

self.drinks_mon = [1500, 1700, 1000]

# Selected order

self.selected_order = {}  # 선택된 주문 항목들을 저장할 딕셔너리

# Menu buttons

self.menu_buttons = {}  # 메뉴 버튼들을 저장

self.create_menu_section("Burgers", self.burgers,self.burgers_mon)  # 햄버거 섹션 생성

self.create_menu_section("Side", self.side,self.side_mon)  # 세트 메뉴 섹션 생성

self.create_menu_section("Drinks", self.drinks,self.drinks_mon)  # 음료 섹션 생성

self.main_layout.addLayout(self.menu_layout)

# 주문 내역 리스트 표시

self.order_list_layout = QVBoxLayout()

self.order_list_label = QLabel("Your Order:")  # "Your Order" 제목 추가

self.order_list_layout.addWidget(self.order_list_label)

self.order_list = QListWidget()  # 주문 리스트

self.order_list_layout.addWidget(self.order_list)

self.main_layout.addLayout(self.order_list_layout)

# Total price display

self.total_price_label = QLabel("Total: $0.00")

self.total_price_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")

self.main_layout.addWidget(self.total_price_label)

# Actions

self.actions_layout = QHBoxLayout()  # 하단 버튼들을 위한 가로 레이아웃

self.clear_all_btn = QPushButton("Clear All")  # 전체 초기화 버튼

self.clear_all_btn.clicked.connect(self.clear_all)  # 초기화 버튼 클릭 시 동작 연결

self.pay_btn = QPushButton("Pay")  # 결제 버튼

self.pay_btn.clicked.connect(self.pay)  # 결제 버튼 클릭 시 동작 연결

self.gpt_btn = QPushButton("GPT")  # GPT 버튼

self.gpt_btn.clicked.connect(self.say_gpt)  # GPT 버튼 클릭 시 동작 연결

# 버튼을 액션 레이아웃에 추가

self.actions_layout.addWidget(self.clear_all_btn)

self.actions_layout.addWidget(self.pay_btn)

self.actions_layout.addWidget(self.gpt_btn)

# 레이아웃을 메인 레이아웃에 추가

self.main_layout.addLayout(self.actions_layout)

# Order display at the bottom

self.setLayout(self.main_layout)  # 전체 레이아웃 설정

def create_menu_section(self, name, items, mons):

layout = QVBoxLayout()  # 각 메뉴 섹션을 위한 세로 레이아웃

layout.addWidget(QLabel(name))  # 섹션 이름 추가

for item, mon in zip(items,mons):

button = QPushButton(f"{item} : {mon}")  # 메뉴 항목 버튼 생성

button.clicked.connect(lambda checked, item=item, mons = mons: self.add_to_order(item,mon))  # 버튼 클릭 시 동작 연결

layout.addWidget(button)

self.menu_layout.addLayout(layout)  # 메뉴 섹션 레이아웃 추가

def add_to_order(self, item, price):

if item in self.selected_order:

self.selected_order[item]['quantity'] += 1

else:

self.selected_order[item] = {'price': price, 'quantity': 1}

self.update_order_list()

def update_order_list(self):

self.order_list.clear()

total_cost = 0

for item, details in self.selected_order.items():

quantity = details['quantity']

price = details['price']

total = quantity * price

total_cost += total

item_widget = QWidget()  # 각 항목을 위젯으로 생성

item_layout = QHBoxLayout()  # 항목 레이아웃

item_label = QLabel(f"{item}: ${total}")  # 항목 이름, 수량, 총 금액 표시

quantity_spinbox = QSpinBox()  # 수량 조절 스핀박스 생성

quantity_spinbox.setRange(0, 99)  # 스핀박스 범위 설정 (0 포함)

quantity_spinbox.setValue(quantity)  # 현재 수량 설정

quantity_spinbox.valueChanged.connect(lambda value, item=item: self.update_item_quantity(item, value))  # 스핀박스 값 변경 시 동작 연결

item_label.setStyleSheet("font-size: 16px; padding: 5px;")

quantity_spinbox.setStyleSheet("font-size: 16px; padding: 5px; width: 70px;")

item_layout.addWidget(item_label)

item_layout.addWidget(quantity_spinbox)

item_widget.setLayout(item_layout)

list_item = QListWidgetItem()

list_item.setSizeHint(item_widget.sizeHint())  # 리스트 항목 크기 설정

self.order_list.addItem(list_item)

self.order_list.setItemWidget(list_item, item_widget)  # 리스트에 항목 추가

self.update_total_price(total_cost)

def update_total_price(self, total_cost):

self.total_price_label.setText(f"Total: ${total_cost}")

def total_count(self, total_price):

self.total_price_label = QLabel(F"Total: ${total_price}")

self.total_price_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")

self.main_layout.addWidget(self.total_price_label)

def update_item_quantity(self, item, quantity):

if quantity == 0:

self.selected_order.pop(item, None)  # 수량이 0이면 항목 삭제

else:

self.selected_order[item]['quantity'] = quantity  # 새로운 수량 설정

self.update_order_list()

def clear_all(self):

# 모든 주문 항목 초기화

self.selected_order.clear()

self.update_order_list()

def pay(self):

if not self.selected_order:

QMessageBox.information(self, "Payment", "No items in your order.")  # 주문 항목이 없을 때 메시지

else:

total_cost = sum(details['price'] * details['quantity'] for details in self.selected_order.values())

QMessageBox.information(self, "Payment", f"Your total is ${total_cost}.\nThank you for your purchase!")  # 주문 요약 메시지

self.clear_all()

def say_gpt(self):

print("녹음 시작")

# RecordAudioWorker를 사용하여 백그라운드에서 녹음 수행

self.audio_worker = RecordAudioWorker(silence_duration=4)

self.audio_worker.recording_done.connect(self.handle_audio_recording_done)

self.audio_worker.start()

def handle_audio_recording_done(self, frames):

audio_file = "output.wav"

save_audio(frames, audio_file)

# GPTWorker를 사용하여 OpenAI API 호출을 백그라운드에서 처리

self.gpt_worker = GPTWorker(audio_file, history_messages, client)

self.gpt_worker.response_ready.connect(self.handle_gpt_response)

self.gpt_worker.error_occurred.connect(self.handle_gpt_error)

self.gpt_worker.start()

def handle_gpt_response(self, response):

print("AI의 응답:", response)

# TTSWorker를 사용하여 TTS 백그라운드 실행

self.tts_worker = TTSWorker(response)

self.tts_worker.start()

# UI에 응답 표시

QMessageBox.information(self, "GPT 응답", response)

def handle_gpt_error(self, error_message):

QMessageBox.critical(self, "오류", error_message)

if __name__ == "__main__":

app = QApplication(sys.argv)  # QApplication 생성

kiosk = BurgerKiosk()  # 키오스크 창 생성

kiosk.show()  # 창 표시

sys.exit(app.exec_())  # 애플리케이션 실행