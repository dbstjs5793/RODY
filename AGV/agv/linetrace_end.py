import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import threading
import cv2.aruco as aruco
import time
import sys
import asyncio
import websockets

async def send_message():
    uri = "ws://172.30.1.31:81"  
    async with websockets.connect(uri) as websocket:
        message = "line_finish"  
        await websocket.send(message)
        print(f"Sent: {message}")


class AGVController:
    def __init__(self, port, baud_rate):
        self.agv = MyAgv(port, baud_rate)
        self.state = False
        self.last_action_time = time.time()
        self.direction, self.turn_cnt, self.find_direction = 0, 0, 0
        self.stop = 0
        self.distance = 0
        self.time=0
        self.cnt=0
        self.marker_length = 0.045 
        self.lock = threading.Lock()  

    def process_frame(self, frame):
        height, width, _ = frame.shape
        roi_height = int(height / 4)
        roi_top = height - roi_height
        roi = frame[roi_top:, :]
        upper_left_roi = roi[:roi_height // 2, :width // 6]
        upper_right_roi = roi[:roi_height // 2, 5 * width // 6:]

        lower_half_roi = roi[roi_height // 2:, :]
        hsv_left = cv2.cvtColor(upper_left_roi, cv2.COLOR_BGR2HSV)
        hsv_right = cv2.cvtColor(upper_right_roi, cv2.COLOR_BGR2HSV)
        hsv_lower = cv2.cvtColor(lower_half_roi, cv2.COLOR_BGR2HSV)

        img_low_1 = np.array([30, 100, 100])
        img_upper_1 = np.array([40, 255, 255])

        yellow_mask_left = cv2.inRange(hsv_left, img_low_1, img_upper_1)
        yellow_mask_right = cv2.inRange(hsv_right, img_low_1, img_upper_1)
        yellow_mask_lower = cv2.inRange(hsv_lower, img_low_1, img_upper_1)

        yellow_result_left = cv2.bitwise_and(upper_left_roi, upper_left_roi, mask=yellow_mask_left)
        yellow_result_right = cv2.bitwise_and(upper_right_roi, upper_right_roi, mask=yellow_mask_right)
        yellow_result_lower = cv2.bitwise_and(lower_half_roi, lower_half_roi, mask=yellow_mask_lower)

        gray_left = cv2.cvtColor(yellow_result_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(yellow_result_right, cv2.COLOR_BGR2GRAY)
        gray_lower = cv2.cvtColor(yellow_result_lower, cv2.COLOR_BGR2GRAY)
        _, binary_image_left = cv2.threshold(gray_left, 50, 255, cv2.THRESH_BINARY)
        _, binary_image_right = cv2.threshold(gray_right, 50, 255, cv2.THRESH_BINARY)
        _, binary_image_lower = cv2.threshold(gray_lower, 50, 255, cv2.THRESH_BINARY)

        contours_left, _ = cv2.findContours(binary_image_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_right, _ = cv2.findContours(binary_image_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_lower, _ = cv2.findContours(binary_image_lower, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(upper_right_roi, contours_right, -1, (255, 0, 0), 2)
        cv2.drawContours(upper_left_roi, contours_left, -1, (0, 255, 0), 2)
        cv2.drawContours(lower_half_roi, contours_lower, -1, (0, 0, 255), 2)
        
        if len(contours_right) >= 1 and self.cnt==0:
            print("R")
            self.cnt=1
            return "RIGHT"
        if len(contours_left) >= 1 and self.cnt==0:
            print("L")
            self.cnt=1
            return "LEFT"
        if len(contours_lower) >= 1:
            max_lower_contour = max(contours_lower, key=cv2.contourArea)
            M = cv2.moments(max_lower_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                center_line = width // 2
                offset = cx - center_line
                self.cnt=0
                if offset < -40:
                    return "Little_left"
                elif -40 <= offset <= 40:
                    return "FORWARD"
                elif offset > 40:
                    return "Little_right"

        return "LOST"
                

    def move_forward(self):
        if time.time() - self.last_action_time > 0.2: 
            self.turn_cnt, self.find_direction = 0, 0
            # self.agv.go_ahead(127, 1)
            self.agv.go_ahead(40, 0.2)
            self.last_action_time = time.time()

    def turn_Little_left(self):
        if time.time() - self.last_action_time > 0.2:
            self.direction, self.turn_cnt, self.find_direction = 1, 0, 0
            self.agv.counterclockwise_rotation(20, 0.2)
            # self.agv.counterclockwise_rotation(90, 1.5)
            self.last_action_time = time.time()

    def turn_left(self):
        if time.time() - self.last_action_time > 0.2:
            self.direction, self.turn_cnt, self.find_direction = 1, 0, 0
            self.agv.go_ahead(15, 2)
            self.agv.counterclockwise_rotation(127, 1.2)
            # self.agv.go_ahead(127, 3)
            # self.agv.counterclockwise_rotation(127, 10)
            self.last_action_time = time.time()

    def turn_Little_right(self):
        if time.time() - self.last_action_time > 0.2:
            self.direction, self.turn_cnt, self.find_direction = 2, 0, 0
            self.agv.clockwise_rotation(20, 0.2)
            # self.agv.clockwise_rotation(90, 1.5)
            self.last_action_time = time.time()

    def turn_right(self):
        if time.time() - self.last_action_time > 0.2:
            self.direction, self.turn_cnt, self.find_direction = 2, 0, 0
            self.agv.go_ahead(15, 1.8)
            self.agv.clockwise_rotation(127, 1.1)
            # self.agv.go_ahead(127, 3)
            # self.agv.clockwise_rotation(127, 10)           
            self.last_action_time = time.time()

    def lost(self):
        if time.time() - self.last_action_time > 0.3:
                self.agv.stop()
                self.agv.go_ahead(10, 2)
                self.agv.stop()
                asyncio.run(send_message())
                sys.exit()

    def stop(self):
        self.agv.stop()
        print("AGV stopped")



    def camera_thread(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera error")
                break

            result = self.process_frame(frame)

            if result == "LEFT":
                self.turn_left()
            elif result == "RIGHT":
                self.turn_right()
            elif result == "Little_left":
                self.turn_Little_left()
            elif result == "Little_right":
                self.turn_Little_right()
            elif result == "FORWARD":
                self.move_forward()
            else:
                self.lost()
            cv2.imshow("Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def start(self):
        self.agv.go_ahead(10, 0.8)
        self.agv.counterclockwise_rotation(60, 2.2)
        self.agv.stop()
        camera_thread = threading.Thread(target=self.camera_thread)
        camera_thread.start()
        camera_thread.join()

if __name__ == "__main__":
    controller = AGVController("/dev/ttyAMA2", 115200)
    controller.start()