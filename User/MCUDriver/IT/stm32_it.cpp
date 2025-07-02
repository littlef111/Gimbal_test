#include "usart.h"
#include "stm32_it.h"

#include "debugc.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

// 没有被使用，等待解决方案(core中的stm32f4xx_it.c的重定义)

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     /* Prevent unused argument(s) compilation warning */
//     // UNUSED(huart);
//     // UNUSED(Size);
//
//     /* NOTE : This function should not be modified, when the callback is needed,
//               the HAL_UARTEx_RxEventCallback can be implemented in the user file.
//      */
//     if (huart == &huart1) {
//         DEBUGC_UartIrqHandler(&huart1);
//     }
//     else if (huart == &huart3) {
//         REMOTEC_UartIrqHandler();
//     }
//     // usart_printf("callback");
// }

