#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <TinyGPS++.h>

// 替换为你的Wi-Fi网络名称和密码
const char* ssid = "666";
const char* password = "zbfql666";

WebSocketsServer webSocket(81); // 初始化WebSocket服务器
#define PIN_PIR 1 // HC-SR501传感器连接的引脚

// 硬件串口配置
#define GPS_RX_PIN 16  // ESP32的GPIO16连接NEO-6M的TX引脚
#define GPS_BAUDRATE 9600

// 创建GPS解析对象
TinyGPSPlus gps;

// 初始化硬件串口2
HardwareSerial gpsSerial(2);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void displayGPSInfo();

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // 等待Wi-Fi连接
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // 启动WebSocket服务器
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // 初始化HC-SR501传感器引脚
  pinMode(PIN_PIR, INPUT);

  // 初始化GPS串口
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, -1);
  
  webSocket.broadcastTXT("\nNEO-6M GPS数据读取示例");
  webSocket.broadcastTXT("等待GPS数据...");
}

void loop() {
  webSocket.loop(); // 处理WebSocket事件

  // 读取并解析所有可用数据
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      displayGPSInfo();
    }
  }

  // 如果长时间没有数据，显示警告
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("未检测到GPS数据，请检查接线！");
    while(true); // 停止运行
  }

  // 读取HC-SR501传感器状态
  int pirState = digitalRead(PIN_PIR); // 替换为HC-SR501连接的引脚

  // 如果传感器状态发生变化，发送更新到所有WebSocket客户端
  if (pirState == HIGH) {
    webSocket.broadcastTXT("Motion detected");
  } else {
    webSocket.broadcastTXT("No motion");
  }

  delay(1000); // 调整延迟以适应你的应用需求
}

void displayGPSInfo() {
  static unsigned long last = 0;
  if (millis() - last < 1000) return; // 每秒更新一次
  last = millis();

  String gpsData = "";

  gpsData += "卫星数: ";
  gpsData += gps.satellites.value();

  if (gps.location.isValid()) {
    gpsData += "\n纬度: ";
    gpsData += gps.location.lat(), 6;
    gpsData += "\n经度: ";
    gpsData += gps.location.lng(), 6;
    gpsData += "\n海拔: ";
    gpsData += gps.altitude.meters();
    gpsData += "m";
    gpsData += "\n速度: ";
    gpsData += gps.speed.kmph();
    gpsData += "km/h";
  } else {
    gpsData += "\n定位中...";
  }

  if (gps.time.isValid()) {
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "\n时间: %02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsData += timeStr;
  }

  if (gps.date.isValid()) {
    char dateStr[32];
    snprintf(dateStr, sizeof(dateStr), "\n日期: %04d-%02d-%02d",
             gps.date.year(), gps.date.month(), gps.date.day());
    gpsData += dateStr;
  }

  // 通过WebSocket广播GPS数据
  webSocket.broadcastTXT(gpsData);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
        }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            // 回显收到的消息
            webSocket.sendTXT(num, "Message received: " + String((char *)payload));
            break;
        case WStype_BIN:
            //Serial.printf("[%u] get binary length: %u\n", num, length);
            //hexdump(payload, length);
            break;
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}
