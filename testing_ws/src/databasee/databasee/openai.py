import openai
import pyaudio
import wave
import speech_recognition as sr
import numpy as np
import time
import pyttsx3  # pyttsx3 라이브러리 임포트

# OpenAI API 키 설정
key = ""
client = openai.OpenAI(api_key=key)

# TTS 엔진 초기화
tts_engine = pyttsx3.init()

# 녹음 설정
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
FRAMES_PER_BUFFER = 1024
SILENCE_DURATION = 1  # 초기 침묵 감지 시간은 5초

# PyAudio 객체 초기화
p = pyaudio.PyAudio()

# "민수야"가 호출되었는지 확인하는 변수
is_called = False
# 음성 인식 실패 횟수를 추적하는 변수
failed_recognition_count = 0
# 대화를 기다리는 상태를 확인하는 변수
waiting_for_response = False

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

def save_audio(frames, filename):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

# SpeechRecognition 객체 초기화
recognizer = sr.Recognizer()

# 음성 파일을 텍스트로 변환 후 OpenAI API와 대화
history_messages = [
    {"role": "system", "content": "당신의 이름은 민수입니다. 세계 최고의 비서로 제가 말하면 요약된 정보로 중요한 것만 이야기를 해주는 유능한 비서입니다."}
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

while True:
    print("녹음 시작...")

    # "민수야"라고 부르기 전에는 침묵 감지 시간 5초
    if not is_called:
        frames = record_audio(silence_duration=1)
    else:
        # "민수야"라고 부르면 침묵 감지 시간을 3초로 변경
        frames = record_audio(silence_duration=3)

    save_audio(frames, "output.wav")

    text = recognize_audio("output.wav")

    if text is None:
        if is_called:
            if not waiting_for_response:
                # 첫 번째 응답 대기 중에 아무 말도 없으면
                #print("죄송한데 한번 더 이야기를 해줄 수 없을까요?")
                #tts_engine.say("죄송한데 한번 더 이야기를 해줄 수 없을까요?")
                tts_engine.runAndWait()
                waiting_for_response = True
            else:
                # 두 번째로 대답이 없을 때 "필요하실 때 부르세요" 출력
                print("필요하실 때 부르세요.")
                tts_engine.say("필요하실 때 부르세요.")
                tts_engine.runAndWait()
                is_called = False
                failed_recognition_count = 0
                waiting_for_response = False
        else:
            tts_engine.runAndWait()
    else:
        print("녹음된 내용:", text)

        if "민수" in text and not is_called:
            # OpenAI ChatCompletion 호출
            history_messages.append({"role": "user", "content": text})
            completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=history_messages,
                temperature=0.2
            )

            answer = completion.choices[0].message.content
            print("AI의 응답:", answer)

            tts_engine.say(answer)
            tts_engine.runAndWait()

            history_messages.append({"role": "assistant", "content": answer})
            is_called = True
            failed_recognition_count = 0  # 초기화
            waiting_for_response = False  # 응답 대기 해제

        elif is_called:
            # 일반적인 대화 응답 처리
            history_messages.append({"role": "user", "content": text})
            completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=history_messages,
                temperature=0.2
            )

            answer = completion.choices[0].message.content
            print("AI의 응답:", answer)

            tts_engine.say(answer)
            tts_engine.runAndWait()

            history_messages.append({"role": "assistant", "content": answer})
            waiting_for_response = False  # 응답 대기 해제

        

# PyAudio 종료
p.terminate()