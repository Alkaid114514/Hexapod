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

// void PCA9685::setPositionRad(uint8_t index, float rad, uint32_t delayMS = 10)
// {
//     float target = ((((rad * 180.0 / PI) / 180) * 2.0 + 0.5) / 20.0) * 4096.0;
//     if (target < SERVO_MIN || target > SERVO_MAX)
//     {
//         return;
//     }
//     if (target <= this->currentPWM[index])
//     {
//         for (int pulseLength = this->currentPWM[index]; pulseLength >= target; --pulseLength)
//         {
//             this->pwmDriver.setPWM(0, 0, pulseLength);
//             delay(delayMS);
//         }
//     }
//     else
//     {
//         for (int pulseLength = this->currentPWM[index]; pulseLength <= target; ++pulseLength)
//         {
//             this->pwmDriver.setPWM(0, 0, pulseLength);
//             delay(delayMS);
//         }
//     }

void PCA9685::setPositionsRadSameTime(uint8_t indices[], float rads[], uint8_t numServos, float rotationTimeMS)
{

    // 初始化每个舵机的目标角度和当前位置
    float targetAngles[numServos];
    float currentAngles[numServos];
    float angleDifferences[numServos];

    // 将目标角度转换为弧度并计算每个舵机的初始角度差值
    for (uint8_t i = 0; i < numServos; ++i)
    {
        uint8_t index = indices[i]; // 当前舵机的索引
        float rad = rads[i];        // 当前舵机的目标角度（弧度）

        // 将弧度转换为角度
        targetAngles[i] = rad * 180.0 / PI;

        // 将 currentPWM[index] 转换为脉冲宽度（单位：ms）
        float pulseWidthCurrent = (this->currentPWM[index] / 4096.0) * 20.0;

        // 将脉冲宽度转换为当前角度
        currentAngles[i] = ((pulseWidthCurrent - 0.5) / 2.0) * 180.0;

        // 计算目标与当前角度之间的差值
        angleDifferences[i] = targetAngles[i] - currentAngles[i];
    }

    // 计算每个舵机每步更新的角度，根据旋转时间
    float totalSteps = rotationTimeMS / 10.0; // 每 10 毫秒进行一次更新
    float stepAngle[numServos];

    // 计算每个舵机每步需要旋转的角度
    for (uint8_t i = 0; i < numServos; ++i)
    {
        stepAngle[i] = angleDifferences[i] / totalSteps;
    }

    // 控制所有舵机的旋转
    unsigned long startTime = millis(); // 记录开始时间
    unsigned long elapsedTime = 0;

    while (elapsedTime < rotationTimeMS)
    {
        bool allFinished = true;

        // 更新每个舵机
        for (uint8_t i = 0; i < numServos; ++i)
        {
            // 计算新的角度
            currentAngles[i] += stepAngle[i];

            // 防止越过目标角度
            if ((angleDifferences[i] > 0 && currentAngles[i] > targetAngles[i]) ||
                (angleDifferences[i] < 0 && currentAngles[i] < targetAngles[i]))
            {
                currentAngles[i] = targetAngles[i];
            }

            // 计算对应的脉冲宽度（0° 对应 0.5ms，180° 对应 2.5ms）
            float pulseWidth = (currentAngles[i] * (2.5 - 0.5)) / 180.0 + 0.5; // 计算脉冲宽度（单位：ms）

            // 将脉冲宽度转换为 PWM 值
            float pwmValue = (pulseWidth / 20.0) * 4096.0; // 将脉冲宽度转换为 PWM 值

            // 设置 PWM 信号
            this->pwmDriver.setPWM(indices[i], 0, pwmValue);
            this->currentPWM[indices[i]] = pwmValue;

            // 判断该舵机是否已到达目标角度
            if (abs(angleDifferences[i]) > 0.1f)
            {
                allFinished = false; // 如果有任一舵机未完成旋转，标记为未完成
            }

            // 更新差值
            angleDifferences[i] = targetAngles[i] - currentAngles[i];
        }

        // 更新经过的时间
        elapsedTime = millis() - startTime;

        // 每 10 毫秒更新一次
        delay(10);

        // 如果所有舵机都完成旋转，则退出
        if (allFinished)
        {
            break;
        }
    }
}

void PCA9685::setPositionsRadSameVelocity(uint8_t indices[], float rads[], uint8_t numServos, float angularVelocity)
{

    // 初始化每个舵机的目标角度和当前位置
    float targetAngles[numServos];
    float currentAngles[numServos];
    float angleDifferences[numServos];

    // 初始化舵机目标角度、当前位置和角度差
    for (uint8_t i = 0; i < numServos; ++i)
    {
        uint8_t index = indices[i]; // 当前舵机的索引
        float rad = rads[i];        // 当前舵机的目标角度（弧度）

        // 将弧度转换为角度
        targetAngles[i] = rad * 180.0 / PI;

        // 将 currentPWM[index] 转换为脉冲宽度（单位：ms）
        float pulseWidthCurrent = (this->currentPWM[index] / 4096.0) * 20.0;

        // 将脉冲宽度转换为当前角度
        currentAngles[i] = ((pulseWidthCurrent - 0.5) / 2.0) * 180.0;

        // 计算目标与当前角度之间的差值
        angleDifferences[i] = targetAngles[i] - currentAngles[i];
    }

    // 控制所有舵机的旋转
    while (true)
    {
        bool allFinished = true;

        // 更新每个舵机
        for (uint8_t i = 0; i < numServos; ++i)
        {
            float stepAngle = angularVelocity * 0.01; // 每 10 毫秒移动的角度，假设 angularVelocity 是每秒旋转的度数

            // 如果角度步长大于剩余角度，则直接调整到目标角度
            if (abs(stepAngle) > abs(angleDifferences[i]))
            {
                stepAngle = angleDifferences[i];
            }

            // 计算新的角度
            currentAngles[i] += stepAngle;

            // 防止越过目标角度
            if ((angleDifferences[i] > 0 && currentAngles[i] > targetAngles[i]) || (angleDifferences[i] < 0 && currentAngles[i] < targetAngles[i]))
            {
                currentAngles[i] = targetAngles[i];
            }

            // 计算对应的脉冲宽度（0° 对应 0.5ms，180° 对应 2.5ms）
            float pulseWidth = (currentAngles[i] * (2.5 - 0.5)) / 180.0 + 0.5; // 计算脉冲宽度（单位：ms）

            // 将脉冲宽度转换为 PWM 值
            float pwmValue = (pulseWidth / 20.0) * 4096.0; // 将脉冲宽度转换为 PWM 值

            // 设置 PWM 信号
            this->pwmDriver.setPWM(indices[i], 0, pwmValue);
            this->currentPWM[indices[i]] = pwmValue;

            // 判断该舵机是否已到达目标角度
            if (abs(angleDifferences[i]) > 0.1f)
            {
                allFinished = false; // 如果有任一舵机未完成旋转，标记为未完成
            }

            // 更新差值
            angleDifferences[i] = targetAngles[i] - currentAngles[i];
        }

        // 如果所有舵机都完成旋转，则退出
        if (allFinished)
        {
            break;
        }

        // 每 10 毫秒更新一次
        delay(10);
    }
}

// void PCA9685::setPositionDeg(uint8_t index, float deg, uint32_t delayMS = 10)
// {

//     float target = (((deg / 180.0) * 2.0 + 0.5) / 20.0) * 4096.0;

//     if (target < SERVO_MIN || target > SERVO_MAX)
//     {
//         return;
//     }
//     if (target <= this->currentPWM[index])
//     {
//         for (int pulseLength = this->currentPWM[index]; pulseLength >= target; --pulseLength)
//         {
//             this->pwmDriver.setPWM(0, 0, pulseLength);
//             delay(delayMS);
//         }
//     }
//     else
//     {
//         for (int pulseLength = this->currentPWM[index]; pulseLength <= target; ++pulseLength)
//         {
//             this->pwmDriver.setPWM(0, 0, pulseLength);
//             delay(delayMS);
//         }
//     }

//     this->currentPWM[index] = target;
// }

void PCA9685::setPositionsDegSameTime(uint8_t indices[], float degs[], uint8_t numServos, float rotationTimeMS)
{
    float rads[numServos];
    for (uint8_t i = 0; i < numServos; ++i)
    {
        rads[i] = degs[i] * PI / 180.0;
    }
    this->setPositionsRadSameTime(indices, rads, numServos, rotationTimeMS);
}

void PCA9685::setPositionsDegSameVelocity(uint8_t indices[], float degs[], uint8_t numServos, float angularVelocity)
{
    float rads[numServos];
    for (uint8_t i = 0; i < numServos; ++i)
    {
        rads[i] = degs[i] * PI / 180.0;
    }
    this->setPositionsRadSameVelocity(indices, rads, numServos, angularVelocity);
}