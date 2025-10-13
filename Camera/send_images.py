import socket
import time
import cv2
import imagezmq

def connect_to_sender():
    """Send a connection attempt to the server and return the ImageSender object if successful."""
    while True:
        try:
            sender = imagezmq.ImageSender(connect_to='tcp://172.30.1.49:5555')
            return sender
        except Exception as e:
            print(f"Failed to connect to server: {e}. Retrying in 30 seconds...")
            time.sleep(10)  # Wait 30 seconds before retrying

# Connect to the image sender
sender = connect_to_sender()  # Attempt to connect initially

rpi_name = socket.gethostname()  # Send RPi hostname with each image

# Open the PiCamera using OpenCV
cap = cv2.VideoCapture(0)  # 0은 기본 카메라 장치를 의미합니다

# 설정: 해상도 320x240, 프레임 속도 10 FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 20)

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("Error: Camera not found!")
    exit()

time.sleep(2.0)  # Allow camera sensor to warm up

while True:  # Send images as stream until Ctrl-C
    try:
        # Capture image frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image!")
            break

        # Send the image along with the Raspberry Pi's hostname
        sender.send_image(rpi_name, frame)
    except Exception as e:
        print(f"Error while sending image: {e}. Trying to reconnect...")
        sender = connect_to_sender()  # Reconnect if sending fails

# Clean up
cap.release()
