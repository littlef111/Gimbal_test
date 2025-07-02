//
// Created by kklit on 25-7-1.
//

#ifndef ITTASK_H
#define ITTASK_H
#ifdef __cplusplus
extern "C" {
#endif

void DEBUGC_UartIrqHandler(UART_HandleTypeDef* huart);
void REMOTEC_UartIrqHandler(void);
void SPI_RxCallBack(void);

#ifdef __cplusplus
}
#endif
#endif //ITTASK_H
