//
// Created by kklit on 25-6-29.
//

#ifndef CORE_H
#define CORE_H

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

void SystemClock_Config(void);
void VisionComTask(void const* argument);
void GimbalControlTask(void const * argument);
void BSP_Init(void);

#endif //CORE_H
