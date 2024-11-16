#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "PCA9685.hpp"

#define SDA_PIN 35
#define SCL_PIN 36

PCA9685 pwm1 = PCA9685(0x40, Wire);
PCA9685 pwm2 = PCA9685(0x41, Wire);

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop()
{
  // Serial.println("Hello World");
  delay(200);
  if (Serial.available() > 0)
  {
    float incomingByte = Serial.parseFloat(); // 读取一个字节
    Serial.println(incomingByte);
    pwm1.setPositionDeg(0, incomingByte);
  }
}

#include <WiFi.h>
#include <WebSocketsServer.h>
// // 替换为你的Wi-Fi网络名称和密码
// const char* ssid = "your_SSID";
// const char* password = "your_PASSWORD";

// WebSocketsServer webSocket(81); // 初始化WebSocket服务器
// #define PIN_PIR 1 // HC-SR501传感器连接的引脚

// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// void setup() {
//   Serial.begin(115200);
//   WiFi.begin("ALKAID-HOME","88888888");

//   // 等待Wi-Fi连接
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("WiFi connected");

//   // 启动WebSocket服务器
//   webSocket.begin();
//   webSocket.onEvent(webSocketEvent);

//   // 初始化HC-SR501传感器引脚
//   pinMode(PIN_PIR, INPUT);
// }

// void loop() {
//   webSocket.loop(); // 处理WebSocket事件

//   // 读取HC-SR501传感器状态
//   int pirState = digitalRead(PIN_PIR); // 替换为HC-SR501连接的引脚

//   // 如果传感器状态发生变化，发送更新到所有WebSocket客户端
//   if (pirState == HIGH) {
//     webSocket.broadcastTXT("Motion detected");
//   } else {
//     webSocket.broadcastTXT("No motion");
//   }

//   delay(100); // 调整延迟以适应你的应用需求
// }
// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
//     switch(type) {
//         case WStype_DISCONNECTED:
//             Serial.printf("[%u] Disconnected!\n", num);
//             break;
//         case WStype_CONNECTED: {
//             IPAddress ip = webSocket.remoteIP(num);
//             Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
//         }
//             break;
//         case WStype_TEXT:
//             Serial.printf("[%u] get Text: %s\n", num, payload);
//             // 回显收到的消息
//             webSocket.sendTXT(num, "Message received: " + String((char *)payload));
//             break;
//         case WStype_BIN:
//             //Serial.printf("[%u] get binary length: %u\n", num, length);
//             //hexdump(payload, length);
//             break;
//         case WStype_ERROR:
//         case WStype_FRAGMENT_TEXT_START:
//         case WStype_FRAGMENT_BIN_START:
//         case WStype_FRAGMENT:
//         case WStype_FRAGMENT_FIN:
//             break;
//     }
// }
