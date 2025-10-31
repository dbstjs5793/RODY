# 🐶 Smart Guide & Storage Robot – **RODY**

> **SLAM + Line Tracing 기반 AGV와 PLC 제어 로봇암을 연동한 스마트 안내·물품보관 로봇 시스템**

![rody](https://github.com/dbstjs5793/RODY/assets/README_MAIN_IMAGE.png)

---

## 📘 Overview

**RODY**는 코엑스 등 대형 복합시설을 대상으로 설계된 **스마트 안내 로봇**입니다.  
운영 시간에는 **길안내 및 물품보관**, 비운영 시간에는 **순찰 및 침입자 감지** 기능을 수행합니다.  
AGV(자율주행차), 로봇암, PLC, Flask Web Server를 통합하여  
**AI 비전 + 로봇 제어 + 통신 구조**를 하나의 시스템으로 구현하였습니다.

---

## 🧭 Key Features

### 🐾 Path Guide (길안내)
- QR코드를 통한 웹페이지 접속 → 출발지·목적지 입력
- SLAM 기반 AGV 자율주행 (쇼핑몰 내부)
- 목적지 도착 시 자동 복귀 및 충전 위치로 이동

### 🎒 Storage Service (물품보관)
- 고객 위치 호출 → AGV가 접근 후 짐 적재
- AGV가 물품보관소로 이동 (라인트레이싱 기반 주행)
- 로봇암이 PLC 레일을 따라 이동하며 짐을 자동 보관/회수

### 🚨 Patrol (순찰)
- 비운영 시간에 YOLOv8 기반 **침입자 감지**
- imagezmq로 AGV→서버 실시간 영상 전송
- Flask 관리자 페이지에서 순찰 이미지 자동 저장 및 날짜별 조회

---

## ⚙️ System Architecture

| 구성 요소 | 주요 역할 | 통신 방식 |
|------------|------------|------------|
| **ESP32 (Main)** | 시스템 중심 제어 및 WebSocket 통신 | Wi-Fi WebSocket |
| **AGV** | SLAM & Line Tracing 주행 | ROS2, WebSocket |
| **로봇암(MyCobot 320)** | ArUco Marker 기반 적재 제어 | RS-232 |
| **PLC (XG5000)** | 레일 및 컨베이어 제어 | RS-485 |
| **Flask Web Server** | 사용자 인터페이스 / 관리자 페이지 | TCP/IP |
| **Raspberry Pi** | 적외선 카메라 영상 수신 및 전송 | imagezmq |

---

## 🧠 SLAM + Line Tracing Hybrid Navigation

- **쇼핑몰 구간:** SLAM(Gmapping + Move_base)  
  → 해상도 0.05m → 0.01m 개선으로 오차율 **15% → 3% 이하 감소**
- **복도–보관소 구간:** Line Tracing  
  → 색상 라인 기반 실시간 조향 제어
- 구간별 주행 전환 로직 및 파라미터 튜닝으로 **메모리 효율 + 주행 안정성 확보**

---

## 👩‍💻 My Role

- 순찰 및 물품보관 기능 **개발 총괄**
- YOLOv8 기반 **Object Detection** 및 imagezmq 실시간 영상 전송 구현
- PLC–로봇암–PC 간 **통신 구조 설계** 및 RS-232·RS-485 기반 제어 로직 개발
- **Aruco Marker 기반 로봇암 좌표 보정** 및 자동 적재 로직 구현
- **WebSocket 프로토콜 설계**로 AGV 도착 신호 기반 로봇암 자동 동작 연동
- **SLAM + Line Tracing 하이브리드 주행 구조 최적화**

---

## 🚀 Results & Performance

- **YOLOv8 침입자 감지 정확도 96% 이상**
- **로봇암 적재 오차 ±2cm 이내 유지**
- **응답 속도 40% 단축**, 안정적 병렬 제어 구조 구현
- **주행 오차율 3% 이하**, 물품 보관·회수 **2분 이내 완료**
- **시스템 확장성 확보:** 스마트 물류/자율주행 시스템으로 확장 가능

---

## 🧩 Tech Stack

**Hardware**  
- MyCobot 320, PLC XG5000, ESP32, Raspberry Pi, IR Camera, AGV

**Software / Libraries**  
- Python · ROS2 · OpenCV · Flask · YOLOv8 · imagezmq · pymycobot  
- RS-232 / RS-485 Serial Communication · WebSocket · SQL · Aruco Marker

---

## 🎥 Demonstration

| 기능 | 영상 | 비고 |
|------|------|------|
| 길안내 (SLAM 주행) | [YouTube Link](https://www.youtube.com/...) | 쇼핑몰 구간 |
| 물품보관 (라인트레이싱+로봇암 제어) | [YouTube Link](https://www.youtube.com/...) | PLC 제어 포함 |
| 순찰 (침입자 감지) | [YouTube Link](https://www.youtube.com/...) | YOLOv8 실시간 감지 |

---

## 🔗 Repository

📁 **GitHub:** [https://github.com/dbstjs5793/RODY](https://github.com/dbstjs5793/RODY)

---

