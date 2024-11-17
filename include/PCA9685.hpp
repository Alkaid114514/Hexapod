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

public:
    PCA9685(int, TwoWire);
    PCA9685(int);
    // void setPositionRad(uint8_t index, float rad, uint32_t delayMS = 10);
    void setPositionsRadSameTime(uint8_t indices[], float rads[], uint8_t numServos, float rotationTimeMS);
    void setPositionsRadSameVelocity(uint8_t indices[], float rads[], uint8_t numServos, float angularVelocity = 10.0f);
    // void setPositionDeg(uint8_t index, float deg, uint32_t delayMS = 10);
    void setPositionsDegSameTime(uint8_t indices[], float degs[], uint8_t numServos, float rotationTimeMS);
    void setPositionsDegSameVelocity(uint8_t indices[], float rads[], uint8_t numServos, float angularVelocity = 10.0f);
};
