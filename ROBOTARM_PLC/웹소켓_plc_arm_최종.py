import serial
import time
import keyboard
from pymycobot.mycobot import MyCobot
import cv2
import numpy as np
import cv2.aruco as aruco
import keyboard
import threading
import websocket

# # MyCobot 포트 설정
mc = MyCobot('COM3', 115200)

#PLC 포트 설정
serial_port = serial.Serial('COM7', baudrate=115200, timeout=1)

# 보정 행렬과 왜곡 계수를 불러옵니다.
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

# ArUco 마커 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.04  # 마커 실제 크기 (미터 단위)

# WebSocket 서버 주소
server_url = "ws://172.30.1.31:84"  # ESP32의 IP 주소와 포트

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 너비 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)   # 높이 설정

# PLC 이동 방향 및 거리
right = [1, 2, 3, 7]
left = [11, 22, 33, 8]

# 사물함 상태를 저장하는 변수 (True = 점유 중, False = 비어 있음)
locker_state = {"1": False, "2": False, "3": False, "4": False, "5": False, "6": True}
id_list = [1,2,3,4,5,6]

def initialize_serial(port: str, baudrate: int = 115200, timeout: float = 2):
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        print(f"Serial port {port} initialized successfully.")
        return ser
    except Exception as e:
        print(f"Error initializing serial port: {e}")
        return None

def display_video():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 항상 비디오 프레임을 화면에 표시
        cv2.imshow("Video", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 사용자가 'q'를 누르면 종료
            break
        
# ArUco 마커를 탐지하여 마커의 ID를 반환하는 함수
def check_aruco_marker_id():
    marker_detected = False  # 탐지 여부를 추적하는 변수
    marker_id = None  # 탐지된 ID를 저장
    
    while not marker_detected:  # marker_detected가 False일 동안 반복
        # print("바구니를 탐지 중...")
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            if ids is not None:
                marker_id = ids[0][0]  # 첫 번째로 탐지된 마커의 ID
                
                if marker_id in id_list:
                    locker_state[str(marker_id)] = True
                    marker_detected = True  # 마커를 찾았으므로 루프 종료 준비
                    print(f"바구니 번호: {marker_id}")
                else: print("유효한 바구니 번호가 아닙니다.")
        # cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 사용자가 'q'를 누르면 종료
            break
    return marker_id

def get_marker_center_relative_to_center(frame, corners):
    """
    ArUco 마커 중심 좌표를 카메라 화면 중심을 (0, 0)으로 기준으로 변환.
    """
    # 카메라 해상도
    frame_height, frame_width = frame.shape[:2]
    
    # 화면 중심 좌표 (0, 0)
    center_x = frame_width // 2
    center_y = frame_height // 2
    
    # ArUco 마커 중심 좌표
    cx = int(corners[:, 0].mean())
    cy = int(corners[:, 1].mean())
    
    # 화면 중심을 기준으로 변환
    relative_cx = cx - center_x
    relative_cy = center_y - cy  # OpenCV 좌표계는 Y축이 아래로 증가하므로 반전 필요
    
    return relative_cx, relative_cy
    
    
def detect_aruco_marker_and_move(mc):
    marker_detected=False
    mc.send_coords([48.3, -221.4, 183.6, -86.38, 0.78, 90.85],60)
    time.sleep(2)
    while(1):
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], marker_length, camera_matrix, dist_coeffs)
                    cx = int(corners[i][0][:, 0].mean())
                    cy = int(corners[i][0][:, 1].mean())
 
                    distance = np.linalg.norm(tvec)
                    aruco.drawDetectedMarkers(frame, corners)
                    cv2.circle(frame, (cx, cy), 10, (255, 0, 0), -1)
   
                    # 변환된 중심 좌표 계산
                    relative_cx, relative_cy = get_marker_center_relative_to_center(frame, corners[i][0])
                    print(f"({relative_cx}, {relative_cy})")
                    
                    y = round(0.3585 * relative_cx - 214.4, 1)
                         
                    cv2.putText(
                        frame,
                        f"Move to ({round(y+214.4, 1)})", (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2
                    )
                    mc.send_coords([47.9,-219.6,226.5,-87.37,-0.05,89.87],60)
                    time.sleep(2)
         
                    print(f"y로 {y}만큼 이동")
                    mc.send_coord(2,y,30)
                    time.sleep(3)    
                                                     
                    print("이동 완료")
                    marker_detected = True  # 마커가 탐지되었음을 설정
                    break
                if marker_detected:  # 마커를 찾았을 경우
                    break  # 외부 루프 종료
            else:
                print("마커 검출 실패, 다시 시도 중...")
        else:
            print("프레임 읽기 실패!")
    # 비디오 화면 표시
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)  # 1ms 대기, 화면 갱신
    return y


def put_box_in_locker(mc, floor):
    if floor == 1:
        # print('1층 넣기 1')
        mc.send_coords([68,-221.1,269.2,-88.19,3.16,88.8],60)
        time.sleep(1)
        # print('1층 넣기 2')
        mc.send_coords([-116.1,-226.1,266.5,-89.93,4.13,90.79],60)
        time.sleep(1)
        # print('1층 넣기 3')
        mc.send_coords([-117,-214.7,227.7,-89.62,2.22,90.46],60)
        time.sleep(0.5)
        # print('1층 빼기')
        mc.send_coords([53.2,-216.3,221.2,-88.06,0.97,90.32],60)
        time.sleep(0.8)
        # print('적재 기본 자세')
        mc.send_angles([-5.36,-115.66,61.25,46.58,-85.51,1.75],60)
        time.sleep(1)
    
    elif floor ==2:
        # print('2층 넣기 1 - 사물함 앞까지')
        mc.send_coords([35.6,-139.8,468,-84.61,2.17,90.75],70)
        time.sleep(1)
        # print('2층 넣기 2 - 사물함 들어가기')
        mc.send_coords([-124.9,-144.8,486.5,-85.18,3.51,91.31],60)
        time.sleep(1)
        # print('그리퍼 내리기')
        mc.send_coords([-124,-150,464.8,-89.53,4.61,91.54],60)
        time.sleep(0.8)
        # print('2층 빼기')
        mc.send_coords([63.6,-143.7,453.4,-88.69,2.46,91.38],60)
        time.sleep(1)
        # print('적재 기본 자세')
        mc.send_angles([-5.36,-115.66,61.25,46.58,-85.51,1.75],70)
        time.sleep(2)
    else:
        print("올바른 층수가 아닙니다.")
    
    return None    
    

def get_box_in_locker(mc, floor):
    if floor == 1:
        y=detect_aruco_marker_and_move(mc)
        # print('1층 밑으로 들어갓')
        mc.send_coords([-130.7,y+10,225.2,-90.19,1.13,92.98],60)
        time.sleep(1.2)
        # print('슬쩍 들어올려보자?')
        mc.send_coords([-130.4,-210.7,267.6,-89.85,3.78,93],60)
        time.sleep(1)
        # print('나와!!!!')
        mc.send_coords([-26,-262.9,254.9,-84.65,5.16,94.36],60)
        time.sleep(1.2)
        # print('나오라고 했다.')
        mc.send_coords([107.1,-229,222,-78.33,2.11,91.38],60)
        time.sleep(0.8)
        # print('적재 기본 자세')
        mc.send_angles([-5.36,-116.1,54.49,46.49,-87.89,-0.43],60)
        time.sleep(2)

    
    elif floor ==2:
        # print('올라가는중~')
        mc.send_coords([63.6,-143.7,453.4,-88.69,2.46,91.38],80)
        time.sleep(1)
        # print('박스 밑으로 들어가!')
        mc.send_coords([-130,-158,470,-90.13,2.18,91.66],60)
        time.sleep(1.5)
        # # print('들어올려!')
        mc.send_coords([-132.7, -137.9, 492.6, -89.92, 3.78, 92.02],60)
        time.sleep(1)
        mc.send_coords([104,-90.8,470,-83.84,1.42,90.44],60)
        time.sleep(1.5)
        # print('적재 기본 자세')
        mc.send_angles([-5.36,-115.66,61.25,46.58,-85.51,1.75],70)
        time.sleep(2)
    else:
        print("올바른 층수가 아닙니다.")
    return None 


##############################################################################################################
#############   물품보관함 상태 관련 함수들 ####################################################################
def check_floor(marker_id):
    floor = None
    if marker_id in [1, 2, 3]:
        floor = 1
    elif marker_id in [4, 5, 6]:
        floor = 2
    else:
        print("floor error")
    return floor

def check_current_locker_state(locker_state):
    print("< 현재 사물함 상태 >")
    for num, state in locker_state.items():
        state = "사용 중" if state else "비어있음"
        print(f"{num}번 사물함 : {state}")
        
def find_empty_locker():
    # 빈 사물함을 찾음
    empty_locker = next((num for num, state in locker_state.items() if not state), None)

    if empty_locker is not None:
        print(f"빈 바구니를 <{empty_locker}>번 사물함에서 꺼냅니다.")
        # 로봇암을 empty_locker 위치로 이동하여 바구니를 꺼내는 동작 수행
        locker_state[empty_locker] = False  # 빈 바구니를 꺼내므로 상태를 False로 유지
        return int(empty_locker)
    else:
        print("모든 사물함이 이용 중입니다.")        
        return None
    
##############################################################################################################
#############   PLC 관련 함수들 ###############################################################################    
def move_plc(position):
    write_to_plc(serial_port, position)
    # PLC가 이동을 완료할 때(=2)까지 대기
    while True:
        # PLC에서 데이터 읽기
        data = read_from_plc(serial_port)
        if data is not None:
            if data == 2:
                break
        else:
            print("No valid data received.")  # 디버깅용 메시지
        time.sleep(0.1)  # 0.1초 대기 (CPU 과부하 방지)
                
    print(f"PC로부터 '{position}'번 신호를 받아 동작 완료")
    # 동작 완료 신호(=9)를 PLC로 전송
    write_to_plc(serial_port, 9)
    
def check_agv_arrival():
    while True:
        # AGV가 도착해 센서에 3초간 인식될 때까지(=1)
        data = read_from_plc(serial_port)
        if data is not None:
            if data == 1:  # 데이터가 1인 경우
                print(f"AGV 도착")
                break
        else:
            print("No valid data received.")  # 디버깅용 메시지
        time.sleep(0.1)  # 대기 (CPU 과부하 방지)
    
def plc_mov_agv_to_locker(marker_id):
    while True:
        # 이전 동작이 완료 되었다는 신호(=1) 읽기
        data = read_from_plc(serial_port)
        if data is not None:
            if data == 1:
                break
        else:
            print("No valid data received.")  # 디버깅용 메시지
        time.sleep(0.1)  # 대기 (CPU 과부하 방지)
    if marker_id in [1, 4]:
        move_plc(right[0])
    elif marker_id in [2, 5]:
        move_plc(right[0])
        move_plc(right[1])
    elif marker_id in [3, 6]:
        move_plc(right[0])
        move_plc(right[2])

def plc_mov_to_emptyID(marker_id, empty_id):
    while True:
        # 이전 동작이 완료 되었다는 신호(=1) 읽기
        data = read_from_plc(serial_port)
        if data is not None:
            if data == 1:
                break
        else:
            print("No valid data received.")  # 디버깅용 메시지
        time.sleep(0.2)  # 대기 (CPU 과부하 방지)
    if marker_id in [1,4]:
        if empty_id in [2, 5]:
            move_plc(right[1])
        elif empty_id in [3, 6]:
            move_plc(right[2])
        else: time.sleep(1)
    elif marker_id in [2, 5]:
        if empty_id in [1, 4]:
            move_plc(left[1])
        elif empty_id in [3, 6]:
            move_plc(right[1])
        else: time.sleep(1)
    elif marker_id in [3, 6]:
        if empty_id in [1, 4]:
            move_plc(left[2])
        elif empty_id in [2, 5]:
            move_plc(left[1])
        else: time.sleep(1)
            
def plc_mov_emptyID_to_agv(empty_id):
    while True:
        # 이전 동작이 완료 되었다는 신호(=1) 읽기
        data = read_from_plc(serial_port)
        if data is not None:
            if data == 1:
                break
        else:
            print("No valid data received.")  # 디버깅용 메시지
        time.sleep(0.2)  # 대기 (CPU 과부하 방지)
    if empty_id in [1, 4]:
        move_plc(left[0])
    elif empty_id in [2, 5]:
        move_plc(left[1])
        move_plc(left[0])
    elif empty_id in [3, 6]:
        move_plc(left[2])
        move_plc(left[0])
    
##############################################################################################################
#############   PLC <-> PC 통신 관련 함수들 ####################################################################    
def read_from_plc(ser: serial.Serial) -> int:
    try:
        # 헤더(1바이트) + 바디(1바이트) + 테일(1바이트) = 총 3바이트 읽기
        raw_data = ser.read(3)
        if len(raw_data) != 3:
            print("Incomplete frame received.")
            return None
        
        # 프레임 구조 확인
        header, body, tail = raw_data
        # 헤더와 테일 검증
        if header != 0x01 and tail != 0x03:  # 수정된 헤더와 테일
            # print(f"Invalid frame received: {raw_data}")
            return None
        # 바디 데이터 반환 (10진수)
        return body
    except Exception as e:
        print(f"Error reading from PLC: {e}")
        return None

def write_to_plc(ser: serial.Serial, data: int):
    try:
        if 0 <= data <= 255:
            frame = bytes([0x01, data, 0x03])
            ser.write(frame)
            time.sleep(0.7)  # PLC의 응답을 기다리기 위해 잠시 대기
        else:
            print(f"Error: Data {data} is out of range (0-255).")
    except Exception as e:
        print(f"Error writing to PLC: {e}")

##############################################################################################################
#############   로봇암 관련 함수들 ############################################################################# 
def arm_in_basic_state(mc):
    # 적재 기본 자세1
    mc.send_coords([228, -176, 294, -86.56, 4.68, 170.64],80)
    time.sleep(1)
    # 적재 기본 자세2
    mc.send_coords([175, -137, 291, -83, 4.41, 131.25],80)
    time.sleep(1)
    
def arm_in_basic_state_reverse(mc):
    # 적재 기본 자세2
    mc.send_coords([175, -137, 291, -83, 4.41, 131.25],80)
    time.sleep(1)
    # 적재 기본 자세1
    mc.send_coords([228, -176, 294, -86.56, 4.68, 170.64],80)
    time.sleep(1)
        
def move_to_origin(mc):
    # AGV에 바구니 넣는 자세
    mc.send_coords([208.7, -177.3, 132.1, -87.34, 3.65, 179.11],80)
    time.sleep(1)

def move_to_origin_box_lift_ver(mc): 
    # 바구니 들기
    mc.send_coords([209, -176.3, 175, -87.63, 2.34, 179.43],80)
    time.sleep(1)
    
def arm_in_camera_position(mc):
    mc.send_coords([206.2, -177.4, 82.7, -87.56, 3.48, 179.1],80)
    time.sleep(1)

def arm_in_push_box_position(mc):
    mc.send_coords([193.1, -150.5, 166.8, -84.7, 2.95, -176.62],80)
    time.sleep(1)

# WebSocket 이벤트 처리
def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed")

def on_open(ws):
    print("Connection opened")
    
#########################################################################################       
# 메시지 처리 함수
def on_message(ws, message):
    # 서버로부터 받은 메시지 출력
    print(f"Server Message: {message}")
    arm_in_camera_position(mc)
    video_thread = threading.Thread(target=display_video)
    video_thread.start()

    # 터미널에서 메시지 입력받기
    message = input("메시지를 입력하세요 (예: 'S', 'R2', 'exit' 등): ")

    ###### 짐 보관 기능 ######
    if message == "S" or message == "s":
        print("짐 보관 기능 수행 중...")
        # AGV 도착 확인
        check_agv_arrival()

        # 바구니 id 및 층수 확인
        load_id = check_aruco_marker_id()
        load_floor = check_floor(load_id)
        
        # # AGV로 이동하여 바구니 꺼내기
        move_to_origin(mc)
        move_plc(left[0])
        move_to_origin_box_lift_ver(mc)
        
        # id에 맞는 곳으로 이동 후 짐 보관
        plc_mov_agv_to_locker(load_id)
        
        if load_id in [1,4]:
            move_plc(right[3])
            arm_in_basic_state(mc)
            move_plc(left[3])
            put_box_in_locker(mc, load_floor)
            locker_state[load_id] = True
        else:
            arm_in_basic_state(mc)
            put_box_in_locker(mc, load_floor)
            locker_state[load_id] = True
            
        # 빈바구니를 찾아 꺼내기
        empty_id = find_empty_locker()
        empty_floor = check_floor(empty_id)
        plc_mov_to_emptyID(load_id, empty_id)
        
        get_box_in_locker(mc, 1)
        if empty_id in [1, 4]:
            move_plc(right[3])
            arm_in_basic_state_reverse(mc)
            move_to_origin_box_lift_ver(mc)
            move_plc(left[3])
        else:
            arm_in_basic_state_reverse(mc)
            move_to_origin_box_lift_ver(mc)
        
        # 빈바구니를 agv에 넣기
        plc_mov_emptyID_to_agv(empty_id)

        move_to_origin(mc)

        # agv에 박스 밀어 넣기
        move_plc(right[0])
        arm_in_push_box_position(mc)
        move_plc(left[3])
        print("짐보관기능 수행완료")
        
        #로봇암 기본 위치로 이동
        move_plc(right[3])
        arm_in_camera_position(mc)

    ###### 짐 찾기 기능 ######
    elif message.startswith("R") or message.startswith("r"):
        load_id = int(message[1:])  # 메시지에서 사물함 번호 추출
        print(f"{load_id}번 사물함에서 짐을 찾는 중...")
        
        ## 짐바구니 위치로 이동
        if load_id in locker_state and locker_state[load_id]:
            # AGV 도착 확인
            check_agv_arrival()

            # 빈바구니 id 및 층수 확인
            empty_id = check_aruco_marker_id()
            empty_floor = check_floor(empty_id)
            
            # AGV로 이동하여 빈바구니 꺼내기
            move_to_origin(mc)
            move_plc(left[0])
            move_to_origin_box_lift_ver(mc)
            
            # id에 맞는 곳으로 이동 후 빈바구니 넣기
            plc_mov_agv_to_locker(empty_id)
            
            if empty_id in [1, 4]:
                move_plc(right[3])
                arm_in_basic_state(mc)
                move_plc(left[3])
                put_box_in_locker(mc, empty_floor)           
                        
            else:
                arm_in_basic_state(mc)
                put_box_in_locker(mc, empty_floor)
                
            # 짐이 들어있는 바구니를 찾아 꺼내기
            load_floor = check_floor(load_id)
            plc_mov_to_emptyID(empty_id, load_id)
            get_box_in_locker(mc, load_floor)
            # 사물함 비우기
            locker_state[load_id] = False
            
            if load_id in [1, 4]:
                move_plc(right[3])
                arm_in_basic_state_reverse(mc)
                move_to_origin_box_lift_ver(mc)
                move_plc(left[3])
            
            else:
                arm_in_basic_state_reverse(mc)
                move_to_origin_box_lift_ver(mc)
            # 짐바구니를 agv에 넣기
            plc_mov_emptyID_to_agv(load_id)
            move_to_origin(mc)

            # agv에 박스 밀어 넣기
            move_plc(right[0])
            arm_in_push_box_position(mc)
            move_plc(left[3])
            print("agv로 이동 완료")
            
            # 반환 완료 신호 전송
            print(f"{load_id}번 사물함에서 짐을 꺼냈습니다.")
            print("짐찾기 수행완료")
            
            # 로봇암 기본 위치로 이동
            move_plc(right[3])
            arm_in_camera_position(mc)
            
        else:
            print(f"{load_id}번 사물함은 비어 있습니다. 찾을 짐이 없습니다.")


    elif message == 'Connected to ESP32_esp WebSocket server!':
        print("서버에 연결 되었습니다.")
        
    # 잘못된 메시지 처리
    else:
        print("잘못된 요청입니다.")
        ws.send("error: 잘못된 요청입니다.")  # 잘못된 요청 신호 전송
    
    # 모든 작업이 끝나면 캡쳐 객체를 해제
    cap.release()
    cv2.destroyAllWindows()

# WebSocket 클라이언트 실행
if __name__ == "__main__":
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(server_url,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.run_forever()
