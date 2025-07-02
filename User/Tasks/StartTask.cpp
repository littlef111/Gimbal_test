//
// Created by kklit on 25-6-30.
//
#include "StartTask.h"
#include "cmsis_os.h"
#include "Core.h"

osThreadId GimbalControlTaHandle1;
osThreadId VisionComTaHandle1;
osThreadId imuTaskHandle1;
uint32_t imuTaskBuffer1[ 1024 ];
osStaticThreadDef_t imuTaskControlBlock1;

void FreeRTOS_Task_Init() {
    /* definition and creation of GimbalControlTa */
    osThreadDef(GimbalControlTa, GimbalControlTask, osPriorityHigh, 0, 2048);
    GimbalControlTaHandle1 = osThreadCreate(osThread(GimbalControlTa), NULL);

    /* definition and creation of VisionComTa */
    osThreadDef(VisionComTa, VisionComTask, osPriorityLow, 0, 1024);
    VisionComTaHandle1 = osThreadCreate(osThread(VisionComTa), NULL);

    /* definition and creation of imuTask */
    osThreadStaticDef(imuTask, INS_task, osPriorityRealtime, 0, 1024, imuTaskBuffer1, &imuTaskControlBlock1);
    imuTaskHandle1 = osThreadCreate(osThread(imuTask), NULL);
}