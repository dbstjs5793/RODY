#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* ssid = "ConnectValue_C402_2G"; // WiFi SSID
const char* password = "CVC402!@#$"; // WiFi Password

IPAddress local_IP( 172, 30, 1, 31 ); // ESP32가 사용할 IP address
IPAddress gateway( 172, 30, 1, 254);    // Gateway IP address (공유기 IP주소)
IPAddress subnet( 255, 255, 255, 0 );   // subnet mask

// 2번 핀을 SelfLED로 정의
#define SelfLEDPIN 2  // GPIO 2번을 SelfLED로 정의
#define ButtonPIN 4    // GPIO 4번을 버튼 입력으로 정의

WebServer server(80); // 웹 서버 객체 생성
WebSocketsServer webSocket = WebSocketsServer(81); // 포트 81에서 WebSocket 서버 생성 (AGV 통신)
WebSocketsServer webSocket_esp = WebSocketsServer(82);  // 포트 82에서 WebSocket 서버 생성 (ESP32 간 통신용)
WebSocketsServer webSocket_web = WebSocketsServer(83); // 포트 83에서 WebSocket 서버 생성 (webpage 통신)
WebSocketsServer webSocket_arm = WebSocketsServer(84);  // 포트 84에서 WebSocket 서버 생성 (ARM 간 통신용)
WebSocketsServer webSocket_rasp = WebSocketsServer(85);  // 포트 85에서 WebSocket 서버 생성 (raspberry pi와 통신용)

uint16_t currentWebSocketClient = 0;  // 현재 연결된 클라이언트의 번호 (agv)
uint16_t currentWebSocketClient_esp = 0;  // ESP32 2와의 WebSocket 클라이언트
uint16_t currentWebSocketClient_web = 0;  // webpage와의 WebSocket 클라이언트
uint16_t currentWebSocketClient_arm = 0;  // webpage와의 WebSocket 클라이언트
uint16_t currentWebSocketClient_rasp = 0;  // rasp와의 WebSocket 클라이언트

volatile bool esp_check_received = false;  // 버튼 상태를 기록하는 변수 (인터럽트에서 사용)
volatile bool agv_check_received = false;  // agv에서 메세지 수신 여부를 기록하는 변수 (인터럽트에서 사용)
volatile bool web_check_received = false;  // web에서 메세지 수신 여부를 기록하는 변수 (인터럽트에서 사용)
volatile bool arm_check_received = false;  // arm에서 메세지 수신 여부를 기록하는 변수 (인터럽트에서 사용)
volatile bool rasp_check_received = false;  // rasp에서 메세지 수신 여부를 기록하는 변수 (인터럽트에서 사용)

String agv_received_message = ""; // agv로부터 수신한 메시지를 저장할 변수
String esp_received_message = ""; // ESP32로부터 수신한 메시지를 저장할 변수
String web_received_message = ""; // web로부터 수신한 메시지를 저장할 변수
String arm_received_message = ""; // arm로부터 수신한 메시지를 저장할 변수
String rasp_received_message = ""; // arm로부터 수신한 메시지를 저장할 변수

// 장소를 처리할 버퍼 변수
String firstLocation = "";  // 첫 번째 장소를 저장
String secondLocation = ""; // 두 번째 장소를 저장

volatile bool buttonPressed = false;  // 버튼 상태를 기록하는 변수 (인터럽트에서 사용)
unsigned long lastButtonPressTime = 0;  // 마지막 버튼 눌린 시간
const unsigned long debounceDelay = 200;  // 입력을 막을 시간 (200ms)
volatile bool arrive_flag = false;  // agv가 안내를 종료했는지 여부 확인
volatile bool RETURN_flag = false;  // 물품 찾기 플래그
volatile bool Back_Base_flag = false;  // 물품 찾기 플래그

void setup() {
  Serial.begin(115200);
  // GPIO2를 OUTPUT으로 설정
  pinMode(SelfLEDPIN, OUTPUT);
  // 고정 ip 설정
  if( !WiFi.config( local_IP, gateway, subnet ) )
  {
    Serial.println( "STA failed to configure" );
  }

  WiFi.mode( WIFI_STA );
  WiFi.begin(ssid, password); // WiFi 연결
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {  // 최대 30초 동안 연결 시도
    delay(1000);
    Serial.print(".");  // Wi-Fi 연결 상태 표시
    attempts++;
  }

  // Wi-Fi 연결 여부 확인
  if (WiFi.status() == WL_CONNECTED) 
  {
    Serial.println("Connected to WiFi");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());  // ESP32 IP 주소 출력
  } 
  else 
  {
    Serial.println("Failed to connect to WiFi.");
  }
  
  // 새로운 '/location' 경로와 핸들러 등록
  server.on("/location", HTTP_GET, handleLocation);
  // 새로운 '/send_message' 경로와 핸들러 등록
  server.on("/send_message", HTTP_POST, handleSendMessage);

  // 서버 시작
  server.begin();
  Serial.println("Server started");

  // WebSocket 핸들러 설정 agv
  webSocket.onEvent(webSocketEvent);
  webSocket.begin();
  // WebSocket 핸들러 설정 esp
  webSocket_esp.onEvent(webSocketEvent_esp);
  webSocket_esp.begin();
  // WebSocket 핸들러 설정 web
  webSocket_web.onEvent(webSocketEvent_web);
  webSocket_web.begin();
  // WebSocket 핸들러 설정 arm
  webSocket_arm.onEvent(webSocketEvent_arm);
  webSocket_arm.begin();
  // WebSocket 핸들러 설정 rasp
  webSocket_rasp.onEvent(webSocketEvent_rasp);
  webSocket_rasp.begin();
}

void loop() {
  // 클라이언트 요청 처리
  server.handleClient();
  webSocket.loop();
  webSocket_esp.loop();
  webSocket_web.loop();
  webSocket_arm.loop();
  webSocket_rasp.loop();
  
  if(esp_check_received) // esp에서 받은 데이터 처리
  {
    esp_check_received = false;
    handleESPReceivedData();
  }
  if(agv_check_received) // agv에서 받은 데이터 처리
  {
    agv_check_received = false;
    handleAGVReceivedData();
  }
  if (arm_check_received) //arm 에서 받은 데이터 처리
  {
    arm_check_received = false;  // 플래그 리셋
    handleARMReceivedData();
  }
  if(web_check_received) //web에서 받은 데이터 처리
  {
    web_check_received = false;
    handleWebReceivedData();
  }
  if(rasp_check_received) //rasp에서 받은 데이터 처리
  {
    rasp_check_received = false;
    handleRaspReceivedData();
  }

  // debug용 ------------------------------------------------------------------------------------------------------------
  // 시리얼 모니터에서 입력받기
  if (Serial.available() > 0) {
    String testmessage = Serial.readStringUntil('\n');  // 엔터 입력 시 문자열 읽기
    testmessage.trim();  // 공백 제거
    sendToClients(testmessage);
  }
  // ----------------------------------------------------------------------------------------------------------------------------
}
// agv 메시지를 전송하는 함수
void sendMessageToAGV(String message) {
  if (currentWebSocketClient != 0 && message.length() > 0) {
    webSocket.sendTXT(currentWebSocketClient - 1, message);  // 받은 메시지 전송
    Serial.println(message + (currentWebSocketClient - 1) + " sent to AGV");
  }
}

// ARM WebSocket 클라이언트로 메시지를 전송하는 함수
void sendMessageToArm(String message) {
  if (currentWebSocketClient_arm != 0) {
    webSocket_arm.sendTXT(currentWebSocketClient_arm - 1, message);  // 메시지를 ARM 클라이언트로 전송
    Serial.println(message + " to ARM");
  }
}

// 일반 WebSocket 클라이언트로 메시지를 전송하는 함수
void sendMessageToWeb(String message) {
  if (currentWebSocketClient_web != 0) {
    webSocket_web.sendTXT(currentWebSocketClient_web - 1, message);  // 메시지를 WebSocket_web 클라이언트로 전송
    Serial.println(message + " to WEB");
  }
}

// rasp websocket 클라이언트로 메시지를 전송하는 함수
void sendMessageToRasp(String message) {
  if (currentWebSocketClient_rasp != 0) {
    webSocket_rasp.sendTXT(currentWebSocketClient_rasp - 1, message);  // 메시지를 WebSocket_web 클라이언트로 전송
    Serial.println(message + " to rasp");
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {

    case WStype_DISCONNECTED:
      Serial.printf("Client agv  %u disconnected\n", num);
      currentWebSocketClient = currentWebSocketClient - 1;  // 클라이언트 연결 해제
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("Client agv %u connected from %d.%d.%d.%d on port 81\n", num, ip[0], ip[1], ip[2], ip[3]);
        webSocket.sendTXT(num, "Welcome to the WebSocket server!");
        currentWebSocketClient = num + 1;  // 현재 연결된 클라이언트 저장
      }
      break;
    case WStype_TEXT:
    if (length == 0) {
        Serial.println("Received empty payload");
        break;  // 길이가 0인 경우 처리
      }
      else {
        Serial.printf("Received text: %s\n", payload);
        agv_received_message = String((char*)payload); // payload를 String으로 변환하여 저장
        agv_check_received = true;
        break;
      }
  }
}

void webSocketEvent_esp(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("ESP32 2 disconnected from WebSocket_esp on port 82\n");
      currentWebSocketClient_esp = currentWebSocketClient_esp - 1;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket_esp.remoteIP(num);
        Serial.printf("ESP32 2 connected from %d.%d.%d.%d to WebSocket on port 82\n", ip[0], ip[1], ip[2], ip[3]);
        webSocket_esp.sendTXT(num, "Connected to ESP32_esp WebSocket server!");
        currentWebSocketClient_esp = num + 1;
      }
      break;
    case WStype_TEXT:
      Serial.printf("Received text esp on port 82: %s\n", payload);
      // 수신한 메시지를 전역 변수에 저장하고 플래그를 true로 설정
      esp_received_message = String((char*)payload); // payload를 String으로 변환하여 저장
      esp_check_received = true;
      break;
  }
}

void webSocketEvent_web(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("web page disconnected from WebSocket_web on port 83\n");
      currentWebSocketClient_web = currentWebSocketClient_web - 1;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket_web.remoteIP(num);
        Serial.printf("web page connected from %d.%d.%d.%d to WebSocket on port 83\n", ip[0], ip[1], ip[2], ip[3]);
        webSocket_web.sendTXT(num, "Connected to ESP32_esp WebSocket server!");
        currentWebSocketClient_web = num + 1;
      }
      break;
    case WStype_TEXT:
      Serial.printf("Received text esp on port 83: %s\n", payload);
      // 수신한 메시지를 전역 변수에 저장하고 플래그를 true로 설정
      web_received_message = String((char*)payload); // payload를 String으로 변환하여 저장
      web_check_received = true;
      break;
  }
}

void webSocketEvent_arm(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("ARM disconnected from WebSocket_web on port 84\n");
      currentWebSocketClient_arm = currentWebSocketClient_arm - 1;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket_arm.remoteIP(num);
        Serial.printf("ARM connected from %d.%d.%d.%d to WebSocket on port 84\n", ip[0], ip[1], ip[2], ip[3]);
        webSocket_arm.sendTXT(num, "Connected to ESP32_esp WebSocket server!");
        currentWebSocketClient_arm = num + 1;
      }
      break;
    case WStype_TEXT:
      // 수신한 메시지를 전역 변수에 저장하고 플래그를 true로 설정
      arm_received_message = String((char*)payload); // payload를 String으로 변환하여 저장
      arm_check_received = true;
      break;
  }
}

void webSocketEvent_rasp(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("web page disconnected from WebSocket_rasp on port 85\n");
      currentWebSocketClient_rasp = currentWebSocketClient_rasp - 1;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket_rasp.remoteIP(num);
        Serial.printf("rasp connected from %d.%d.%d.%d to WebSocket on port 85\n", ip[0], ip[1], ip[2], ip[3]);
        webSocket_rasp.sendTXT(num, "Connected to ESP32_esp WebSocket server!");
        currentWebSocketClient_rasp = num + 1;
      }
      break;
    case WStype_TEXT:
      // 수신한 메시지를 전역 변수에 저장하고 플래그를 true로 설정
      rasp_received_message = String((char*)payload); // payload를 String으로 변환하여 저장
      rasp_check_received = true;
      break;
  }
}

// 장소 정보 요청을 처리하는 함수
void handleLocation() {
  String area = server.arg("area");  // 요청에서 지역 정보 가져오기
  Serial.println(area + " handleLocation 입니다.");  // 터미널에 지역 정보 출력
  server.sendHeader("Access-Control-Allow-Origin", "*");  // CORS 헤더 추가

  int spaceIndex = area.indexOf(" ");  // 첫 번째 공백의 위치 찾기
  int commaIndex = area.indexOf(",");  // 첫 번째 쉼표의 위치 찾기

  // 받은 지역 정보가 쉼표가 포함되어 있을 경우
  if (commaIndex != -1) {
    firstLocation = area.substring(0, commaIndex);  // 첫 번째 장소
    secondLocation = area.substring(commaIndex + 1);  // 두 번째 장소
    sendMessageToAGV(firstLocation);  // 첫 번째 장소 전송
  } 
  // 쉼표 없이 공백이 포함되어 있을 경우
  else if (spaceIndex != -1) {
    firstLocation = area.substring(0, spaceIndex);  // 첫 번째 장소
    secondLocation = area.substring(spaceIndex + 1);  // 두 번째 장소

    if (firstLocation == "RETURN") {  // 물품 찾기
      RETURN_flag = true;
      sendMessageToAGV("save");  // "save" 전송
    } else { // SAVE 들어올때 : 물품 보관
      sendMessageToAGV(secondLocation);  // 두 번째 장소 전송
    }
  }
  else {
    sendMessageToAGV(area);  // Item_Storage, Navigation
  }
  

  // 기본 "go" 메시지 전송
  if (area != "Navigation" && area != "Item Storage") {
    sendMessageToAGV("go");
  }

  server.send(200, "text/plain", area + " 정보가 ESP32에 전달되었습니다.");  // 클라이언트에게 응답 전송
}

// 메시지 요청을 처리하는 함수
void handleSendMessage() {
  String message = server.arg("plain");  // POST 요청에서 메시지 내용 가져오기
  Serial.println("Received message: " + message);  // 터미널에 메시지 출력
  server.sendHeader("Access-Control-Allow-Origin", "*");  // CORS 헤더 추가

  // "R"로 시작하는 메시지 처리
  if (message.startsWith("R")) {
    handleRMessage(message);  // "R"로 시작하는 메시지 처리 함수 호출
  } else {
    sendMessageToAGV(message);  // 그 외의 메시지 AGV 전송
  }

  server.send(200, "text/plain", "Message received: " + message);  // 클라이언트에게 응답 전송
}

// "R"로 시작하는 메시지 처리 함수
void handleRMessage(String message) {
  String numberPart = message.substring(1);  // "R" 뒤의 숫자 부분 추출
  int number = numberPart.toInt();  // 숫자로 변환

  if (number >= 1 && number <= 6) {
    Serial.println("Received valid number: " + String(number));
    sendMessageToArm(message);  // WebSocket을 통해 ARM에 메시지 전송
  }
}

void handleESPReceivedData() {
  // "stop" 메시지 처리
  if (esp_received_message == "stop" && arrive_flag) {
    arrive_flag = false;
  } else {
    // Web으로 전송
    if (esp_received_message.length() > 0) {
      sendMessageToWeb(esp_received_message); // 웹 클라이언트로 메시지 전송
    }
    // AGV로 전송
    if (esp_received_message.length() > 0) {
      sendMessageToAGV(esp_received_message); // AGV 클라이언트로 메시지 전송
    }
  }
}

void handleAGVReceivedData() {
  // 특정 메시지에 따른 처리를 위한 코드
  if (agv_received_message == "start") {
    if (secondLocation.length() > 0) {
      sendMessageToAGV(secondLocation); // AGV 클라이언트로 메시지 전송
      secondLocation = "";
    }
  } 
  else if (agv_received_message == "arrive") {
    arrive_flag = true;
    sendMessageToAGV("base"); // AGV 클라이언트로 메시지 전송
    delay(100);
    sendMessageToAGV("go"); // AGV 클라이언트로 메시지 전송
  } 
  else if (agv_received_message == "store") {
    sendMessageToRasp(agv_received_message);
  } 
  else if (agv_received_message == "line_start") {
    delay(1000);
    sendMessageToAGV("start_line_trace"); // AGV 클라이언트로 메시지 전송
  } 
  else if (agv_received_message == "line_finish") {
    if (RETURN_flag) {  // 물품 찾기 때 
      sendMessageToAGV(secondLocation); // AGV 클라이언트로 메시지 전송
      secondLocation = "";
      RETURN_flag = false;
    }
    else if (Back_Base_flag) {
      sendMessageToAGV("base"); // AGV 클라이언트로 메시지 전송
      delay(100);
      sendMessageToAGV("go"); // AGV 클라이언트로 메시지 전송
      Back_Base_flag = false;
    }
    else {
      sendMessageToRasp("Open");
    }
  }
  else if (agv_received_message == "retrieve") {
    sendMessageToRasp(agv_received_message);
  }
  else if (agv_received_message == "servo") {
    sendMessageToRasp(agv_received_message);
  }
  else {
    Serial.println("Unknown message: " + agv_received_message);
  }

  sendMessageToWeb(agv_received_message); // 웹 클라이언트로 메시지 전송
}


void handleARMReceivedData() {
  // "S" 메시지 처리
  if (arm_received_message.startsWith("S")) {
    String numberPart = arm_received_message.substring(1);
    int number = numberPart.toInt();

    if (number >= 1 && number <= 6) {
      Serial.println("Received valid number: " + String(number));
      sendMessageToWeb(arm_received_message);  // Web 클라이언트로 메시지 전송
      sendMessageToRasp("Close");
      delay(50);
      sendMessageToAGV("Back");  // AGV 클라이언트로 "Back" 메시지 전송
      Back_Base_flag = true;
    } else {
      Serial.println("Invalid message, expected S1 to S6. Received: " + arm_received_message);
    }
  }
  // "Back" 메시지 처리
  else if (arm_received_message.startsWith("B")) {
    sendMessageToRasp("Close");
    Back_Base_flag = true;
    delay(50);
    sendMessageToAGV(arm_received_message);  // AGV 클라이언트로 "Back" 메시지 전송
  } else {
    Serial.println("Received ARM message: " + arm_received_message);
  }
}

void handleWebReceivedData() {
  // "cancel" 메시지 처리
  if (web_received_message == "cancel") {
    Serial.println("cancel received by WEB");
    sendMessageToAGV(web_received_message);  // AGV 클라이언트로 메시지 전송
  } else {
    Serial.println("알 수 없는 메시지 수신 by web");
  }
}

void handleRaspReceivedData() {
  // "retrieve" 메시지 처리
  if (rasp_received_message == "retrieve") {
    Serial.println(rasp_received_message + " received by Rasp");
    sendMessageToAGV("base"); // AGV 클라이언트로 메시지 전송
    delay(100);
    sendMessageToAGV("go"); // AGV 클라이언트로 메시지 전송
  } else if (rasp_received_message == "store"){
    Serial.println(rasp_received_message + " received by Rasp");
    delay(1000);
    sendMessageToAGV("save"); // AGV 클라이언트로 메시지 전송
    delay(500);
    sendMessageToAGV("go"); // AGV 클라이언트로 메시지 전송
    sendMessageToArm("S"); // ARM 클라이언트로 메시지 전송 
  }
  else{
    Serial.println(rasp_received_message + " by rasp");
  }
}

// 디버깅용 메시지를 각 WebSocket 클라이언트에 전송하는 함수
void sendToClients(const String& testmessage) {
  // AGV 클라이언트로 메시지 전송
  sendMessageToAGV(testmessage);

  // ARM 클라이언트로 메시지 전송
  sendMessageToArm(testmessage);

  // 웹 클라이언트로 메시지 전송
  sendMessageToWeb(testmessage);

  // rasp 클라이언트로 메시지 전송
  sendMessageToRasp(testmessage);
}
