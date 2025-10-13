#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Arduino.h>

const char* ssid = "ConnectValue_C402_2G";  // WiFi SSID
const char* password = "CVC402!@#$";  // WiFi Password

#define SelfLEDPIN 2  // GPIO 2번을 SelfLED로 정의
#define ButtonPIN 4  // 버튼 핀 정의
#define ESP32_SERVER_IP "172.30.1.31"  // WebSocket 서버의 IP 주소

WebSocketsClient webSocket;  // WebSocket 클라이언트 객체

volatile bool buttonStateChanged = false;  // 버튼 상태 변화 플래그
volatile bool buttonStatePressed = false;  // 버튼 상태 변화 플래그
unsigned long lastButtonChangeTime = 0;   // 마지막 상태 변경 시간 추적
const unsigned long debounceDelay = 100;   // 디바운스 시간 (100ms)
const unsigned long longPressDelay = 5000; // 긴 버튼 누름 체크 (5초)

int buttonPressCount = 0;  // 버튼 눌린 횟수
int buttonReleaseCount = 0;  // 버튼 떼어진 횟수
unsigned long lastDebounceTime = 0;  // 디바운스 체크 시간

volatile bool first_sent = false;  // 첫 번째 'go' 전송 여부 체크
unsigned long buttonPressStartTime = 0; // 버튼 눌림 시작 시간

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long currentTime = millis();
  // 디바운스 처리
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonStateChanged = true;  // 버튼 상태가 변경됨
    buttonStatePressed = true;
    lastDebounceTime = currentTime;  // 마지막 디바운스 시간 갱신
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ButtonPIN, INPUT_PULLUP);  // 버튼 입력 핀 설정
  pinMode(SelfLEDPIN, OUTPUT);      // SelfLED 출력 핀 설정

  // 버튼 상태 변화에 대한 인터럽트 설정 (CHANGE)
  attachInterrupt(digitalPinToInterrupt(ButtonPIN), handleButtonInterrupt, CHANGE);

  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi.");
  }

  // WebSocket 클라이언트 연결
  webSocket.begin(ESP32_SERVER_IP, 82, "/");  // 고정 IP 주소 사용
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();  // WebSocket 메시지 처리
  if(buttonStatePressed){
    if (buttonStateChanged) {
      // 버튼 상태를 10ms마다 10번 읽어서 최종 상태 결정
      buttonPressCount = 0;
      buttonReleaseCount = 0;

      // 10번 버튼 상태를 읽음
      for (int i = 0; i < 10; i++) {
        int buttonState = digitalRead(ButtonPIN);  // 버튼 상태 읽기
        if (buttonState == LOW) {
          buttonPressCount++;  // 버튼이 눌렸으면 버튼 눌림 횟수 증가
        } else {
          buttonReleaseCount++;  // 버튼이 떼어졌으면 버튼 떼어짐 횟수 증가
        }
        delay(10);  // 10ms 대기
      }

      // 버튼 눌림 횟수와 떼어짐 횟수를 비교하여 메시지 전송
      if (buttonPressCount > buttonReleaseCount) {
        // 버튼이 더 많이 눌렸으면 'go' 메시지 전송
        if (!first_sent) {
          first_sent = true;
          buttonPressStartTime = millis();
          if (webSocket.isConnected()) {
            webSocket.sendTXT("go");
            Serial.println("Sent 'go' to WebSocket server.");
          }
        }

        // 5초 이상 버튼을 누르고 있으면 'keep go' 전송
        // if(first_sent){
        //   if (millis() - buttonPressStartTime > longPressDelay) {
        //     buttonPressStartTime = millis();
        //     if (webSocket.isConnected()) {
        //       webSocket.sendTXT("keep go");
        //       Serial.println("Sent 'keep go' to WebSocket server.");
        //     }
        //   }
        // }
      } else {
        // 버튼이 더 많이 떼어졌으면 'stop' 메시지 전송
        if (webSocket.isConnected()) {
          webSocket.sendTXT("stop");
          Serial.println("Sent 'stop' to WebSocket server.");
        }

        // 'stop' 메시지를 보낸 후 플래그 초기화
        first_sent = false;  // 처음 1회 'go' 보내주는 변수 초기화
        buttonStateChanged = false;  // 상태 변경 플래그 초기화
        buttonStatePressed = false;
      }
    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server.");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server.");
      break;
    case WStype_TEXT:
      Serial.printf("Received message: %s\n", payload);
      break;
  }
}
