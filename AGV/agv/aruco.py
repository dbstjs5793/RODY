import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import threading
import cv2.aruco as aruco
import time
import sys

class AGVController:
    def __init__(self, port, baud_rate):
        self.agv = MyAgv(port, baud_rate)
        self.state = False
        self.last_action_time = time.time()
        self.direction, self.turn_cnt, self.find_direction = 0, 0, 0
        self.stop = 0
        self.distance = 0
        self.time = 0
        self.cnt = 0
        self.marker_length = 0.045
        self.go_state = 0
        self.lock = threading.Lock()

    def check(self, frame):
        camera_matrix = np.load(r"/home/er/ey_ws/Image/camera_matrix.npy")
        dist_coeffs = np.load(r"/home/er/ey_ws/Image/dist_coeffs.npy")
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, camera_matrix, dist_coeffs)
                distance = np.linalg.norm(tvec)
                return ids[i][0], rvec, tvec, distance
        return None, None, None, None

    def aruco(self, ids, rvec, tvec, distance):
        if ids == 0:
            # print(f"Marker 7 detected. Distance: {distance:.2f}m. Moving forward...")
            
            if time.time() - self.last_action_time > 0.3:
                if self.go_state == 0:
                    if tvec is not None:
                        x = tvec[0][0][0]  
                        if x < -0:
                            print(f"x={x:.2f}. right")
                            self.agv.counterclockwise_rotation(10, 0.3)  
                        elif x > 0.17:
                            print(f"x={x:.2f}. left")
                            self.agv.clockwise_rotation(10, 0.3)  
                        else:
                            print(f"x={x:.2f}. go_start")
                            self.go_state = 1
                        self.last_action_time = time.time()
                        
                else:
                    if distance < 0.4:
                        self.agv.counterclockwise_rotation(60, 4)
                        self.go_state = 0
                        self.last_action_time = time.time()
                        return "end"
                    else:
                        self.agv.go_ahead(10, 0.3)
                        self.last_action_time = time.time()
        # elif ids==7:
        #     if time.time() - self.last_action_time > 0.2:
                
        #         if self.go_state == 0:
        #             if tvec is not None:
        #                 x = tvec[0][0][0]  
        #                 print(f"x={x:.3f}. right")
        #                 # time.sleep(1)
        #                 if x < -0.01:
        #                     # print(f"x={x:.2f}. right")
        #                     self.agv.counterclockwise_rotation(5, 0.2)  # ������ ȸ��
        #                 elif x > 0.01:
        #                     # print(f"x={x:.2f}. left")
        #                     self.agv.clockwise_rotation(5, 0.2)  # ���� ȸ��
        #                 else:
        #                     # print(f"x={x:.2f}. go_start")
        #                     self.go_state = 1
        #                 self.last_action_time = time.time()
                        
        #         else:
        #             if 0.40 > distance:
        #                 self.agv.retreat(5, 0.2)
        #                 self.last_action_time = time.time()
                        
        #             elif distance>0.43:
        #                 self.agv.go_ahead(5, 0.2)
        #                 self.last_action_time = time.time()
        #             else:
        #                 self.agv.counterclockwise_rotation(60, 2.2)
        #                 self.agv.retreat(10, 0.8)
        #                 self.go_state = 0
        #                 self.last_action_time = time.time()
        #                 return "end"
            # time.sleep(3)
        return "none"

    def camera_thread(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        self.running = True
        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("Camera error")
                break

            maker, rvec, tvec, dis = self.check(frame)
            if maker is not None:
                text_to_display = f"Marker: {maker}, Distance: {dis:.2f}m"
                result = self.aruco(maker, rvec, tvec, dis)
                if result == "end":
                    # print("Task completed. Stopping camera thread.")
                    self.running = False
            else:
                text_to_display = "No Marker"

            cv2.putText(frame, text_to_display, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                # print("User requested exit. Stopping camera thread.")
                self.running = False

        cap.release()
        cv2.destroyAllWindows()
        # sys.exit() 

    def start(self):
        camera_thread = threading.Thread(target=self.camera_thread)
        camera_thread.start()
        camera_thread.join()

if __name__ == "__main__":
    controller = AGVController("/dev/ttyAMA2", 115200)
    controller.start()