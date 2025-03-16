#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>
#include <ArduinoJson.h>

// 替换为你的Wi-Fi网络名称和密码
const char* ssid = "ceshi";
const char* password = "zbfql666";

WebSocketsServer webSocket(81); // 初始化WebSocket服务器
#define PIN_PIR 1 // HC-SR501传感器连接的引脚

// 硬件串口配置
#define GPS_RX_PIN 16  // ESP32的GPIO16连接NEO-6M的TX引脚
#define GPS_BAUDRATE 9600
#define SDAPIN 21  // 接对应引脚
#define SCLPIN 20

unsigned long lastSensor1=0; 

Adafruit_MPU6050 mpu;
sensors_event_t a,w,t,g;
  // 互补滤波参数
  float alpha = 0.98; // 融合系数
  float pitch = 0;    // 俯仰角
  float roll = 0;     // 横滚角

// 创建GPS解析对象
TinyGPSPlus gps;

// 初始化硬件串口2
HardwareSerial gpsSerial(2);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void displayGPSInfo();

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Wire.begin(SDAPIN, SCLPIN);
  mpu.begin(0x68, &Wire,SCLPIN);


  // 等待Wi-Fi连接
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(WiFi.status());
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
  if (millis() - lastSensor1 >= 300) {
    int pirState = digitalRead(PIN_PIR);
    DynamicJsonDocument doc(128);
    doc["type"] = "Sensor1";
    doc["data"] = (pirState == HIGH) ? "检测到运动！" : "未探测到运动";
    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
    lastSensor1 = millis();  // 更新时间戳
  }
  mpu.getEvent(&a, &w, &t);
  // 计算加速度计的姿态角
  float accel_pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accel_roll = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

  // 计算陀螺仪的角速度积分
  float gyro_pitch = pitch + g.gyro.y * 0.01; // 0.01 是时间间隔（假设 10ms）
  float gyro_roll = roll + g.gyro.x * 0.01;

  // 互补滤波融合加速度计和陀螺仪数据
  pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;
  roll = alpha * gyro_roll + (1 - alpha) * accel_roll;
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 10) { 
    DynamicJsonDocument doc(256);
    doc["type"] = "Sensor3";
    JsonObject data = doc.createNestedObject("data");
    data["pitch"] = pitch;
    data["roll"] = roll;

    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
    lastSend = millis();
  }
  delay(10); // 调整延迟以适应你的应用需求
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
  //webSocket.broadcastTXT(String("{\"type\":\"Sensor2\",\"data\":\"") + gpsData + "\"}");
// 将 GPS 数据构造为 JSON 对象

DynamicJsonDocument doc(512);
doc["type"] = "Sensor2";

JsonObject data = doc.createNestedObject("data"); // 嵌套对象
data["satellites"] = gps.satellites.value();

if (gps.location.isValid()) {
  data["latitude"] = gps.location.lat();
  data["longitude"] = gps.location.lng();
  data["altitude"] = gps.altitude.meters();
  data["speed"] = gps.speed.kmph();
} else {
  data["status"] = "定位中...";
}

String output;
serializeJson(doc, output);
webSocket.broadcastTXT(output);
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
