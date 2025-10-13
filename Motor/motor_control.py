import sys
sys.path.append('/home/er/.local/lib/python3.10/site-packages')  # 실제 설치 경로로 변경

import websocket
import threading
import time
import RPi.GPIO as GPIO# WebSocket 서버 주소

GPIO_1 = 12
GPIO_2 = 13
GPIO_3 = 18

server_url = "ws://172.30.1.31:85"  # ESP32의 IP 주소와 포트 (예: ws://192.168.1.100:81)# GPIO 핀 설정
GPIO.setmode(GPIO.BCM)  # GPIO 핀 번호 방식 설정 (BCM 방식)
GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정


def control_servo(ws, message):
    # 서브모터 18번 제어
    GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정

    pwm_GPIO_1 = GPIO.PWM(GPIO_1, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_2 = GPIO.PWM(GPIO_2, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_3 = GPIO.PWM(GPIO_3, 50)  # 50Hz의 주파수로 PWM 객체 생성
    
    pwm_GPIO_1.start(0)  # PWM 초기화 (0% 듀티 사이클)
    pwm_GPIO_2.start(0)
    pwm_GPIO_3.start(0)

    GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정

    time.sleep(1)  # 1초 대기 (서브모터 초기화)
    # 서브모터 1번 제어 (2.5%에서 10.0%로 증가, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
        pwm_GPIO_1.ChangeDutyCycle(2.5 + i * 1.0)  # 2.5%에서 10%로 증가
        time.sleep(0.1)  # 0.1초마다 값 증가
        GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    time.sleep(1)  # 1초 대기
    # 서브모터 2번과 3번 제어 (2.5% -> 7.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
        pwm_GPIO_2.ChangeDutyCycle(7.5 - i * 0.5)  # 7.5%에서 2.5%로 감소
        pwm_GPIO_3.ChangeDutyCycle(2.5 + i * 0.5)  # 2.5%에서 7.5%로 증가
        time.sleep(0.1)
        GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
    time.sleep(1)  # 1초 대기
    # 서브모터 2번과 3번을 반대로 설정 (7.5% -> 2.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
        pwm_GPIO_2.ChangeDutyCycle(2.5 + i * 0.5)  # 2.5%에서 시작해서 0.5씩 증가
        pwm_GPIO_3.ChangeDutyCycle(7.5 - i * 0.5)  # 7.5%에서 시작해서 0.5씩 감소
        time.sleep(0.1)
        GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수

    time.sleep(1)  # 1초 대기

    pwm_GPIO_2.stop()  # PWM 정지    
    pwm_GPIO_3.stop()  # PWM 정지    

    time.sleep(60)  # 서브모터 16번 초기화 대기
    
    # 서브모터 1번 원위치로 돌아가게 설정 (10%에서 2.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
        pwm_GPIO_1.ChangeDutyCycle(12.5 - i * 1.0)  # 10%에서 2.5%로 감소
        time.sleep(0.1)
        GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    
    time.sleep(1)  # 1초 대기
    pwm_GPIO_1.stop()  # PWM 정지    ws.send(message)  # 메시지 전송# 웹소켓에서 메시지를 받을 때 호출되는 함수

    ws.send(message)

def open_servo(ws, message):
    # 서브모터 18번 제어
    GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정

    pwm_GPIO_1 = GPIO.PWM(GPIO_1, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_2 = GPIO.PWM(GPIO_2, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_3 = GPIO.PWM(GPIO_3, 50)  # 50Hz의 주파수로 PWM 객체 생성
    
    pwm_GPIO_1.start(0)  # PWM 초기화 (0% 듀티 사이클)
    pwm_GPIO_2.start(0)
    pwm_GPIO_3.start(0)

    GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정

    time.sleep(1)  # 1초 대기 (서브모터 초기화)
    # 서브모터 1번 제어 (2.5%에서 10.0%로 증가, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
        pwm_GPIO_1.ChangeDutyCycle(2.5 + i * 1.0)  # 2.5%에서 10%로 증가
        time.sleep(0.1)  # 0.1초마다 값 증가
        GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    time.sleep(1)  # 1초 대기
    # 서브모터 2번과 3번 제어 (2.5% -> 7.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
        pwm_GPIO_2.ChangeDutyCycle(7.5 - i * 0.5)  # 7.5%에서 2.5%로 감소
        pwm_GPIO_3.ChangeDutyCycle(2.5 + i * 0.5)  # 2.5%에서 7.5%로 증가
        time.sleep(0.1)
        GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
    time.sleep(1)  # 1초 대기
    # 서브모터 2번과 3번을 반대로 설정 (7.5% -> 2.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수
        pwm_GPIO_2.ChangeDutyCycle(2.5 + i * 0.5)  # 2.5%에서 시작해서 0.5씩 증가
        pwm_GPIO_3.ChangeDutyCycle(7.5 - i * 0.5)  # 7.5%에서 시작해서 0.5씩 감소
        time.sleep(0.1)
        GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
        GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정# 서브모터 제어 함수

    time.sleep(1)  # 1초 대기

    pwm_GPIO_2.stop()  # PWM 정지    
    pwm_GPIO_3.stop()  # PWM 정지    

    ws.send(message)

def close_servo(ws, message):
    # 서브모터 18번 제어
    GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.OUT)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.OUT)  # GPIO18번 핀을 출력으로 설정

    pwm_GPIO_1 = GPIO.PWM(GPIO_1, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_2 = GPIO.PWM(GPIO_2, 50)  # 50Hz의 주파수로 PWM 객체 생성
    pwm_GPIO_3 = GPIO.PWM(GPIO_3, 50)  # 50Hz의 주파수로 PWM 객체 생성
    
    pwm_GPIO_1.start(0)  # PWM 초기화 (0% 듀티 사이클)
    pwm_GPIO_2.start(0)
    pwm_GPIO_3.start(0)

    GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    GPIO.setup(GPIO_2, GPIO.IN)  # GPIO17번 핀을 출력으로 설정
    GPIO.setup(GPIO_3, GPIO.IN)  # GPIO18번 핀을 출력으로 설정

    time.sleep(1)  # 1초 대기 (서브모터 초기화)
    
    # 서브모터 1번 원위치로 돌아가게 설정 (10%에서 2.5%로 변화, 1초 동안)
    for i in range(1,11):  # 10번의 반복으로 1초 동안 변화
        GPIO.setup(GPIO_1, GPIO.OUT)  # GPIO16번 핀을 출력으로 설정
        pwm_GPIO_1.ChangeDutyCycle(12.5 - i * 1.0)  # 10%에서 2.5%로 감소
        time.sleep(0.1)
        GPIO.setup(GPIO_1, GPIO.IN)  # GPIO16번 핀을 출력으로 설정
    
    time.sleep(1)  # 1초 대기
    pwm_GPIO_1.stop()  # PWM 정지    ws.send(message)  # 메시지 전송# 웹소켓에서 메시지를 받을 때 호출되는 함수

    ws.send(message)

def on_message(ws, message):
    print("Received:", message)    # 메시지에 따라 서브모터 제어
    if message == "retrieve" or message == "store" or message == "servo":
        # 서브모터 동작 후, 서브모터가 끝난 뒤 메시지 보내기
        time.sleep(1)  # 2초 대기 (서버로부터 받은 메시지 후 대기 시간)
        control_servo(ws, message)
    elif message == "Open":
        # 서브모터 동작 후, 서브모터가 끝난 뒤 메시지 보내기
        time.sleep(1)  # 2초 대기 (서버로부터 받은 메시지 후 대기 시간)
        open_servo(ws, message)
    elif message == "Close":
        # 서브모터 동작 후, 서브모터가 끝난 뒤 메시지 보내기
        time.sleep(1)  # 2초 대기 (서버로부터 받은 메시지 후 대기 시간)
        close_servo(ws, message)

def on_error(ws, error):
    print("Error:", error)
def on_close(ws):
    print("Connection closed")
def on_open(ws):
    print("Connection opened")

if __name__ == "__main__":
    websocket.enableTrace(False)  # 디버그 모드 활성화
    ws = websocket.WebSocketApp(server_url,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    # WebSocket 클라이언트 실행
    ws.run_forever()

    # GPIO 리소스 해제
    GPIO.cleanup()