#!/usr/bin/env python
import numpy as np
import math
import time
import threading
import cv2
import cv2.aruco as aruco
import rospy
from geometry_msgs.msg import Twist

# ���Ͽ��� ī�޶� ��İ� �ְ� ��� �ε�
camera_matrix = np.load(r"/home/er/ey_ws/Image/camera_matrix.npy")
dist_coeffs = np.load(r"/home/er/ey_ws/Image/dist_coeffs.npy")

cam = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX  # �ؽ�Ʈ ���÷��̿� ��Ʈ
ret, frame = cam.read()
width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
count = cam.get(cv2.CAP_PROP_FRAME_COUNT)
fps = cam.get(cv2.CAP_PROP_FPS)
size = frame.shape
focal_length = size[1]
center = (size[1] / 2, size[0] / 2)
# ī�޶� ���� �Ķ����
camera_matrix = np.array(
[[focal_length, 0, center[0]], [0, focal_length, center[1]],
[0, 0, 1]],
dtype="double")
cam.release()

# �Ʒ��� ���� ����
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
marker_length = 0.055  # ����: ����
aruco_params = cv2.aruco.DetectorParameters_create()

# ȸ�� ��� ���� ����
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

pose_data = [None, None, None, None, None, None]
_id = [0]
pose_data_dict={}

x = 0
y = 0
theta = 0

def go(x, y, theta):
    rospy.init_node('qcode_detect', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = theta
    pub.publish(twist)

def _is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def _rotation_matrix_to_euler_angles(R):
    assert (_is_rotation_matrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def _detect(corners, ids, imgWithAruco):
    if len(corners) > 0:
        x1 = (int(corners[0][0][0][0]), int(corners[0][0][0][1]))
        x2 = (int(corners[0][0][1][0]), int(corners[0][0][1][1]))
        x3 = (int(corners[0][0][2][0]), int(corners[0][0][2][1]))
        x4 = (int(corners[0][0][3][0]), int(corners[0][0][3][1]))

        cv2.line(imgWithAruco, x1, x2, (255, 0, 0), 1)
        cv2.line(imgWithAruco, x2, x3, (255, 0, 0), 1)
        cv2.line(imgWithAruco, x3, x4, (255, 0, 0), 1)
        cv2.line(imgWithAruco, x4, x1, (255, 0, 0), 1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(imgWithAruco, 'C1', x1, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(imgWithAruco, 'C2', x2, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(imgWithAruco, 'C3', x3, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(imgWithAruco, 'C4', x4, font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        if ids is not None:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for i in range(rvec.shape[0]):
                cv2.aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, marker_length)
                aruco.drawDetectedMarkers(imgWithAruco, corners)

            cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4), int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))
            cv2.putText(frame, "id=" + str(ids), cornerMid, font, 1, (255, 255, 255), 1, cv2.LINE_AA)

            rvec = rvec[0][0]
            tvec = tvec[0][0]

            str_position = "MARKER Position x=%.4f (cm)  y=%.4f (cm)  z=%.4f (cm)" % (tvec[0] * 100, tvec[1] * 100, tvec[2] * 100)

            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)

            str_attitude = "MARKER Attitude degrees r=%.4f  p=%.4f  y=%.4f" % (math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker))

            print(str_position)
            print("rotation x=%.4f (degree) " % (math.degrees(math.atan(tvec[0]/tvec[2]))))
            print(str_attitude)
            print(math.degrees(pitch_marker) + math.degrees(math.atan(tvec[0]/tvec[2])))
            print("-----------------------------------------------")

            pose_data[0] = tvec[0] * 100
            pose_data[1] = tvec[1] * 100
            pose_data[2] = tvec[2] * 100
            pose_data[3] = math.degrees(roll_marker)
            pose_data[4] = math.degrees(pitch_marker)
            pose_data[5] = math.degrees(yaw_marker)

            pose_data_dict[ids] = pose_data
            return (tvec[0] * 100, tvec[1] * 100, tvec[2] * 100), \
                   (math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker))
    else:
        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
        pose_data[3] = None
        pose_data_dict[0] = pose_data
        return None

def main():
    degrees = 15
    degrees2 = 3.5
    fdegrees2 = 0
    while True:
        cam = cv2.VideoCapture('/dev/video0')
        ret, frame = cam.read()
        cam.release()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        imgWithAruco = frame.copy()
        pose, angle = _detect(corners, ids, imgWithAruco)

        if pose is not None:
            x = pose[0]
            y = pose[1]
            theta = math.radians(angle[2])

            if ids[0] == 1:
                go(x, y, theta)
            print(f" : x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

            # ���ǿ� �´� �̵� ���� �߰�
            if theta > degrees:
                fdegrees2 += degrees2
            if x < degrees:
                fdegrees2 -= degrees2
            if y > degrees2:
                go(x + degrees, y + degrees2, fdegrees2)
        else:
            print("!")
            pass

        time.sleep(0.5)