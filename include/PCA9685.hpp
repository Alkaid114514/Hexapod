#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVO_MIN (((0.0 / 180.0) * 2.0 + 0.5) / 20.0) * 4096.0
#define SERVO_MAX (((180.0 / 180.0) * 2.0 + 0.5) / 20.0) * 4096.0
#define SERVO_FREQUENCY 50

class PCA9685
{
private:
    Adafruit_PWMServoDriver pwmDriver;
    float currentPWM[16];
    /* data */
public:
    PCA9685(int, TwoWire);
    PCA9685(int);
    void setPositionRad(uint8_t index, float rad, uint32_t delayMS = 10);
    void setPositionDeg(uint8_t index, float deg, uint32_t delayMS = 10);
};
