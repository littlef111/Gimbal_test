#ifndef ROBO_TEST_CORE_INC_UART8_H_
#define ROBO_TEST_CORE_INC_UART8_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

#define DEBUG_RVSIZE 255

class DebugC {
private:
    float vel_kp;
    float vel_ki;
    float vel_kd;
    int32_t vel_maxOutput;
    int32_t vel_maxIntegral;
    int32_t vel_rampTargetValue;
    int32_t vel_rampTargetTime;
    int32_t vel_rampTargetStep;

    float pos_kp;
    float pos_ki;
    float pos_kd;
    int32_t pos_maxOutput;
    int32_t pos_maxIntegral;
    int32_t pos_maxOutStep;
    int32_t pos_targetAngle;

public:
    // 速度环参数访问方法
    float getVelKp() const { return vel_kp; }
    void setVelKp(float value) { vel_kp = value; }
    
    float getVelKi() const { return vel_ki; }
    void setVelKi(float value) { vel_ki = value; }
    
    float getVelKd() const { return vel_kd; }
    void setVelKd(float value) { vel_kd = value; }
    
    int32_t getVelMaxOutput() const { return vel_maxOutput; }
    void setVelMaxOutput(int32_t value) { vel_maxOutput = value; }
    
    int32_t getVelMaxIntegral() const { return vel_maxIntegral; }
    void setVelMaxIntegral(int32_t value) { vel_maxIntegral = value; }
    
    int32_t getVelRampTargetValue() const { return vel_rampTargetValue; }
    void setVelRampTargetValue(int32_t value) { vel_rampTargetValue = value; }
    
    int32_t getVelRampTargetTime() const { return vel_rampTargetTime; }
    void setVelRampTargetTime(int32_t value) { vel_rampTargetTime = value; }
    
    int32_t getVelRampTargetStep() const { return vel_rampTargetStep; }
    void setVelRampTargetStep(int32_t value) { vel_rampTargetStep = value; }

    // 位置环参数访问方法
    float getPosKp() const { return pos_kp; }
    void setPosKp(float value) { pos_kp = value; }
    
    float getPosKi() const { return pos_ki; }
    void setPosKi(float value) { pos_ki = value; }
    
    float getPosKd() const { return pos_kd; }
    void setPosKd(float value) { pos_kd = value; }
    
    int32_t getPosMaxOutput() const { return pos_maxOutput; }
    void setPosMaxOutput(int32_t value) { pos_maxOutput = value; }
    
    int32_t getPosMaxIntegral() const { return pos_maxIntegral; }
    void setPosMaxIntegral(int32_t value) { pos_maxIntegral = value; }
    
    int32_t getPosMaxOutStep() const { return pos_maxOutStep; }
    void setPosMaxOutStep(int32_t value) { pos_maxOutStep = value; }
    
    int32_t getPosTargetAngle() const { return pos_targetAngle; }
    void setPosTargetAngle(int32_t value) { pos_targetAngle = value; }
};

#define STARTPID  0x70
#define STARTLQR 0x6C
#define START 0x31
#define STOP  0x30
#define MAOHAO  0x3A

#define VEL_LOOP 0x73
#define VEL_KP 0x70
#define VEL_KI 0x69
#define VEL_KD 0x64
#define VEL_MAXOUT 0x6F
#define VEL_MAXINTEGRAL 0x61
#define VEL_TARVALUE 0x76
#define VEL_TARTIME 0x74
#define VEL_TARSTEP 0x73

#define POS_LOOP 0x70
#define POS_KP 0x70
#define POS_KI 0x69
#define POS_KD 0x64
#define POS_MAXOUT 0x6F
#define POS_MAXINTEGRAL 0x61
#define POS_MAXSTEP 0x73
#define POS_TARVALUE 0x76

extern DebugC debugParam;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void DEBUGC_UartInit();
void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart);

#endif