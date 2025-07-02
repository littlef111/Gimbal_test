#include "debugc.h"
#include <cstring>
#include <cstdlib>
#include "usart.h"
#include "usartio.h"
#include "ITtask.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

DebugC debugParam;

#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];

char debugRvBuff[DEBUG_RVSIZE] = { 0 };
char debugBuff[DEBUG_RVSIZE] = { 0 };
char *pEnd;
int16_t start_flag = 0;

void DEBUGC_UartIdleCallback(UART_HandleTypeDef* huart);

void DEBUGC_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}

void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)send_buf, length);
}

void usart_printf_3pin(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)send_buf, length);
}

void DEBUGC_UartIrqHandler(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)!= RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            DEBUGC_UartIdleCallback(huart);
        }
    }
}

void DEBUGC_UartIdleCallback(UART_HandleTypeDef* huart)
{
    HAL_UART_DMAStop(huart);
    memcpy(debugBuff, &debugRvBuff[5], 10);
    uint8_t data_length = DEBUG_RVSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (debugRvBuff[5] == MAOHAO && debugRvBuff[6] == START)
    {
        start_flag = 1;
    }
    else if (debugRvBuff[5] == MAOHAO && debugRvBuff[6] == STOP) start_flag = 0;

    switch (debugRvBuff[0])
    {
    case VEL_LOOP:
    {
        switch (debugRvBuff[3])
        {
        case VEL_KP:
            debugParam.setVelKp(strtof(debugBuff, &pEnd));
            break;
        case VEL_KI:
            debugParam.setVelKi(strtof(debugBuff, &pEnd));
            break;
        case VEL_KD:
            debugParam.setVelKd(strtof(debugBuff, &pEnd));
            break;
        case VEL_MAXOUT:
            debugParam.setVelMaxOutput(strtof(debugBuff, &pEnd));
            break;
        case VEL_MAXINTEGRAL:
            debugParam.setVelMaxIntegral(strtof(debugBuff, &pEnd));
            break;
        case VEL_TARVALUE:
            debugParam.setVelRampTargetValue(strtof(debugBuff, &pEnd));
            break;
        case VEL_TARTIME:
            debugParam.setVelRampTargetTime(strtof(debugBuff, &pEnd));
            break;
        case VEL_TARSTEP:
            debugParam.setVelRampTargetStep(strtof(debugBuff, &pEnd));
            break;
        }
    }
    case POS_LOOP:
    {
        switch (debugRvBuff[3])
        {
        case POS_KP:
            debugParam.setPosKp(strtof(debugBuff, &pEnd));
            break;
        case POS_KI:
            debugParam.setPosKi(strtof(debugBuff, &pEnd));
            break;
        case POS_KD:
            debugParam.setPosKd(strtof(debugBuff, &pEnd));
            break;
        case POS_MAXOUT:
            debugParam.setPosMaxOutput(strtof(debugBuff, &pEnd));
            break;
        case POS_MAXINTEGRAL:
            debugParam.setPosMaxIntegral(strtof(debugBuff, &pEnd));
            break;
        case POS_MAXSTEP:
            debugParam.setPosMaxOutStep(strtof(debugBuff, &pEnd));
            break;
        case POS_TARVALUE:
            debugParam.setPosTargetAngle(strtof(debugBuff, &pEnd));
            break;
        }
    }
    }
    memset(debugRvBuff, 0, data_length);
    data_length = 0;
    HAL_UART_Receive_DMA(huart, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}