from flask import Flask, render_template, request, jsonify, Response
from flask_cors import CORS
import pymysql
import time
import cv2
import imagezmq
from ultralytics import YOLO
from datetime import datetime
import threading  # 스레드 관련 모듈 추가
import os
from collections import defaultdict  # 날짜별로 이미지 그룹화에 사용

# Flask 앱 설정
app = Flask(__name__)
CORS(app)

# YOLO 모델 로드
model = YOLO("yolov8s.pt", verbose=False)

# ImageHub로부터 이미지를 받을 준비
image_hub = imagezmq.ImageHub(open_port="tcp://*:5555")  # TCP 포트 5555

# 전역 변수로 이미지를 저장할 변수
image = None
image_paths = []  # 저장된 이미지 파일 경로를 저장할 리스트
# Lock 객체 생성
image_lock = threading.Lock()

# 사람 감지 신뢰도 기준 설정
CONFIDENCE_THRESHOLD = 0.9

# DB 연결 함수
def get_db_connection():
    return pymysql.connect(host='172.30.1.49', user='kgDB', password='kgstudent', db='kgdb', charset='utf8')

@app.route('/')
def basic():
    return render_template("page_main.html")

@app.route('/page_navigation')
def page_two():
    return render_template("page_navigation.html")

@app.route('/page_item_storage')
def page_three():
    return render_template("page_item_storage.html")

@app.route('/admin_login')
def page_admin_login():
    return render_template("page_admin_login.html")

@app.route('/admin')
def page_admin():
    return render_template("page_admin.html")

# 비밀번호 저장, 검증, 삭제 관련 API들...
@app.route('/page_item_storage/save_password', methods=['POST'])
def save_password():
    data = request.json
    password = data.get('password')
    
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute("INSERT INTO customer_luggage (customer_password) VALUES (%s)", (password,))
        conn.commit()
        message = "비밀번호가 성공적으로 저장되었습니다!"
    except Exception as e:
        conn.rollback()
        message = f"저장 실패: {str(e)}"
    finally:
        cur.close()
        conn.close()

    return jsonify({"message": message})

@app.route('/page_item_storage/verify_password', methods=['POST'])
def verify_password():
    data = request.json
    password = data.get('password')

    conn = get_db_connection()
    cur = conn.cursor()
    try:
        # 비밀번호가 존재하는지 확인
        cur.execute("SELECT COUNT(*) FROM customer_luggage WHERE customer_password = %s", (password,))
        count = cur.fetchone()[0]

        if count > 0:
            return jsonify({"valid": True})
        else:
            return jsonify({"valid": False})

    except Exception as e:
        return jsonify({"error": str(e)})
    finally:
        cur.close()
        conn.close()

@app.route('/page_item_storage/delete_password', methods=['POST'])
def delete_password():
    data = request.json
    password = data.get('password')

    conn = get_db_connection()
    cur = conn.cursor()
    try:
        # 비밀번호 삭제 쿼리
        cur.execute("DELETE FROM customer_luggage WHERE customer_password = %s", (password,))
        conn.commit()

        # 삭제된 행 수 확인
        if cur.rowcount > 0:
            message = "비밀번호가 성공적으로 삭제되었습니다!"
        else:
            message = "비밀번호를 찾을 수 없습니다."

    except Exception as e:
        conn.rollback()
        message = f"삭제 실패: {str(e)}"
    finally:
        cur.close()
        conn.close()

    return jsonify({"message": message})

@app.route('/page_item_storage/save_storage_location', methods=['POST'])
def save_storage_location():
    data = request.json
    customer_password = data.get('customer_password')  # 비밀번호
    storage_location = data.get('storage_location')  # 물품 보관 위치
    
    conn = get_db_connection()
    try:
        # 비밀번호로 고객을 찾고 해당 비밀번호에 대한 물품 보관 위치 업데이트
        cur = conn.cursor()
        cur.execute("""
            UPDATE customer_luggage 
            SET storage_location = %s 
            WHERE customer_password = %s
        """, (storage_location, customer_password))
        
        # 변경된 행의 수가 0이면 고객을 찾지 못한 것이므로 예외 처리
        if cur.rowcount == 0:
            return jsonify({"message": "해당 비밀번호로 등록된 고객이 없습니다!"})
        
        conn.commit()
        cur.close()
        return jsonify({"message": "물품 보관 위치가 성공적으로 저장되었습니다!"})
    except Exception as e:
        conn.rollback()
        return jsonify({"message": f"저장 실패: {str(e)}"})
    finally:
        conn.close()

        
@app.route('/page_item_storage/get_storage_location', methods=['POST'])
def get_storage_location():
    data = request.json
    password = data.get('password')

    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute("SELECT storage_location FROM customer_luggage WHERE customer_password = %s", (password,))
        result = cur.fetchone()
        if result:
            storage_location = result[0]
            return jsonify({"storage_location": storage_location})
        else:
            return jsonify({"message": "No storage location found"}), 404
    except Exception as e:
        return jsonify({"message": f"Error: {str(e)}"}), 500
    finally:
        cur.close()
        conn.close()


@app.route('/admin_login/verify', methods=['POST'])
def verify_admin():
    data = request.json
    admin_id = data.get('admin_id')
    admin_password = data.get('admin_password')

    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute("SELECT COUNT(*) FROM admin_table WHERE admin_id = %s AND admin_password = %s", (admin_id, admin_password))
        count = cur.fetchone()[0]

        if count > 0:
            return jsonify({"valid": True})
        else:
            return jsonify({"valid": False})

    except Exception as e:
        print(f"Error: {str(e)}")  # 에러 메시지 출력
        return jsonify({"error": str(e)})
    finally:
        cur.close()
        conn.close()

@app.route('/admin_login/save', methods=['POST'])
def save_admin():
    data = request.json
    admin_id = data.get('admin_id')
    admin_password = data.get('admin_password')

    conn = get_db_connection()
    cur = conn.cursor()
    try:
        cur.execute("INSERT INTO admin_table (admin_id, admin_password) VALUES (%s, %s)", (admin_id, admin_password))
        conn.commit()
        message = "관리자 정보가 성공적으로 저장되었습니다!"
    except Exception as e:
        conn.rollback()
        message = f"저장 실패: {str(e)}"
    finally:
        cur.close()
        conn.close()

    return jsonify({"message": message})

# 이미지를 화면에 띄우는 함수 (추가적 디버깅용)
def show_image(image, window_name):
    try:
        # 이미지를 띄우고 'q'를 눌러 종료 가능하게
        cv2.imshow(window_name, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("q pressed, closing image window.")
            cv2.destroyAllWindows()
    except Exception as e:
        print(f"Error while displaying image: {e}")

# 이미지 수신 및 YOLO 모델을 통한 사람 감지 후, 스트리밍 전송하는 함수
def imagezmq_thread():
    global image
    global image_paths  # 전역 변수로 image_paths 사용
    while True:
        try:
            # 이미지 수신
            rpi_name, new_image = image_hub.recv_image()

            if new_image is None:
                time.sleep(1)
                continue

            # 받은 이미지를 전역변수에 저장 (Lock으로 동기화)
            with image_lock:
                image = new_image

            # YOLO 모델로 사람 감지
            results = model.predict(source=new_image, show=False, device="cpu", classes=0, verbose=False)

            # 감지된 객체가 사람인지 확인
            person_detected = False
            for box in results[0].boxes:
                if box.conf >= CONFIDENCE_THRESHOLD:
                    person_detected = True
                    break

            # 사람 감지 시, 이미지 저장
            if person_detected:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                
                base_dir = os.getcwd()  # 현재 작업 디렉토리 (예: project)
                save_dir = os.path.join(base_dir, 'project_web', 'web_esp32', 'flask', 'static', 'images')
                
                # 폴더가 존재하지 않으면 생성
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)

                # 파일 경로 설정
                filename = os.path.join(save_dir, f"person_detected_{timestamp}.jpg")
                cv2.imwrite(filename, new_image)
                print(f"사람이 감지되었습니다. 이미지 저장: {filename}")

            # 서버에 확인 응답 보내기
            image_hub.send_reply(b'OK')

        except Exception as e:
            print(f"이미지 수신 중 오류 발생: {e}")
            time.sleep(1)  # 수신 실패 시 잠시 대기

@app.route('/admin/person_recognition')
def person_recognition():
    # 이미지가 저장된 폴더 경로
    image_dir = os.path.join(os.getcwd(), 'project_web', 'web_esp32', 'flask', 'static', 'images')

    # 이미지 경로 목록 생성 (폴더 내의 .jpg 파일들만)
    image_paths = []
    try:
        # 디렉토리 내의 파일들 리스트 가져오기
        for filename in os.listdir(image_dir):
            if filename.endswith('.jpg'):
                image_paths.append(f'static/images/{filename}')
    except Exception as e:
        print(f"Error reading image directory: {e}")

    # 이미지 경로 리스트를 JSON 형식으로 반환
    return jsonify({'image_paths': image_paths})

# 날짜별 이미지 파일 반환 API 추가
@app.route('/admin/get_image_dates')
def get_image_dates():
    # 이미지가 저장된 폴더 경로
    image_dir = os.path.join(os.getcwd(), 'project_web', 'web_esp32', 'flask', 'static', 'images')
    # 이미지 파일들을 날짜별로 그룹화
    image_dates = defaultdict(list)
    try:
        # 디렉토리 내의 파일들 리스트 가져오기
        for filename in os.listdir(image_dir):
            if filename.endswith('.jpg'):
                date_str = filename.split('_')[2]+filename.split('_')[3]  # 이미지 이름에서 날짜 부분 추출 (YYYYMMDD)
                # 파일 이름에서 '.jpg' 확장자를 제거
                date_str = os.path.splitext(date_str)[0]
                image_dates[date_str].append(f'static/images/{filename}')
        #         print(date_str)
        # print(image_dates)
    except Exception as e:
        print(f"Error reading image directory: {e}")
    
    # 날짜별로 그룹화된 이미지 경로 반환
    return jsonify({'image_dates': dict(image_dates)})

# 비디오 스트리밍을 위한 라우트
@app.route('/admin/video_feed')
def video_feed():
    # print("video feed 호출")
    
    # 이미지가 수신될 때까지 대기 후 바로 전송
    while image is None:
        time.sleep(0.1)

    try:
        # Lock을 사용하여 최신 이미지 가져오기
        with image_lock:
            current_image = image.copy()

        # 이미지를 스트리밍 형식으로 전송
        _, buffer = cv2.imencode('.jpg', current_image)
        frame = buffer.tobytes()

        # 웹페이지에 JPEG 이미지 스트리밍
        return Response(generate_frame(frame),
                        mimetype='multipart/x-mixed-replace; boundary=frame',
                        headers={"Cache-Control": "no-cache, no-store, must-revalidate"})  # 캐시 방지 헤더 추가

    except Exception as e:
        print(f"Error during video stream: {e}")
        return "Error"

def generate_frame(frame):
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# 이미지 수신과 Flask 앱 실행을 위한 스레드
if __name__ == '__main__':
    # Flask 서버 실행을 위한 스레드
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False, threaded=True))
    flask_thread.start()

    # 이미지 수신을 위한 스레드 실행
    imagezmq_thread = threading.Thread(target=imagezmq_thread)
    imagezmq_thread.start()
