#include "pca9685.hpp"

PCA9685::PCA9685(int address, TwoWire wire)
{
    this->pwmDriver = Adafruit_PWMServoDriver(address, wire);
    this->pwmDriver.begin();
    this->pwmDriver.setOscillatorFrequency(27000000);
    this->pwmDriver.setPWMFreq(SERVO_FREQUENCY);
}

PCA9685::PCA9685(int address)
{
    this->pwmDriver = Adafruit_PWMServoDriver(address);
    this->pwmDriver.begin();
    this->pwmDriver.setOscillatorFrequency(27000000);
    this->pwmDriver.setPWMFreq(SERVO_FREQUENCY);
}

void PCA9685::setPositionRad(uint8_t index, float rad, uint32_t delayMS = 10)
{
    float target = (((rad * 180.0 / PI) * 2.0 + 0.5) / 20.0) * 4096.0;
    if (target < SERVO_MIN || target > SERVO_MAX)
    {
        return;
    }
    if (target <= this->currentPWM[index])
    {
        for (int pulseLength = this->currentPWM[index]; pulseLength >= target; --pulseLength)
        {
            this->pwmDriver.setPWM(0, 0, pulseLength);
            delay(delayMS);
        }
    }
    else
    {
        for (int pulseLength = this->currentPWM[index]; pulseLength <= target; ++pulseLength)
        {
            this->pwmDriver.setPWM(0, 0, pulseLength);
            delay(delayMS);
        }
    }

    this->currentPWM[index] = target;
}

void PCA9685::setPositionDeg(uint8_t index, float deg, uint32_t delayMS = 10)
{

    float target = (((deg / 180.0) * 2.0 + 0.5) / 20.0) * 4096.0;

    if (target < SERVO_MIN || target > SERVO_MAX)
    {
        return;
    }
    if (target <= this->currentPWM[index])
    {
        for (int pulseLength = this->currentPWM[index]; pulseLength >= target; --pulseLength)
        {
            this->pwmDriver.setPWM(0, 0, pulseLength);
            delay(delayMS);
        }
    }
    else
    {
        for (int pulseLength = this->currentPWM[index]; pulseLength <= target; ++pulseLength)
        {
            this->pwmDriver.setPWM(0, 0, pulseLength);
            delay(delayMS);
        }
    }

    this->currentPWM[index] = target;
}