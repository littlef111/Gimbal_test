//
// Created by LENOVO on 2022/9/6.
//
//云台CAN通信反馈信息处理
#include <cstring>
#include "drv_can.h"
#include "debugc.h"
#include "../../Tasks/VISIONCOM_Task/visioncom_task.h"
#include "shootc.h"
#include "gimbalc.h"
#include "INS_task.h"


JudgeMessage judge;
CAN can;

///CAN自动选择邮箱发送
uint8_t CAN_TxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* pHeader, uint8_t aData[])
{
    uint8_t mailbox = 0;
    uint32_t pTxMailbox[mailbox];
    if (hcan->Instance->TSR & (1 << 26)) mailbox = 0;       //邮箱0为空
    else if (hcan->Instance->TSR & (1 << 27)) mailbox = 1;  //邮箱1为空
    else if (hcan->Instance->TSR & (1 << 28)) mailbox = 2;  //邮箱2为空
    else return 0xFF;                                   //无空邮箱,无法发送
    hcan->Instance->sTxMailBox[mailbox].TIR = 0;        //清除之前的设置
    HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
    return mailbox;
}

void Motor::clear() {
    LastAngel = 0;
    NowAngel = 0;
    IncreAngel = 0;
    MotorAngel_ALL = 0;
    Angel_Ecd = 0;
    Speed_Ecd = 0;
    Angel_All = 0;
    Motor_Speed = 0;
}

void Motor::update_angle() {
    LastAngel = NowAngel;
    NowAngel = Angel;
    IncreAngel = GetEncoderContinueAngel();
    MotorAngel_ALL += IncreAngel;
    Angel_Ecd = MotorAngel_ALL * 360.0f / (EncoderPerCircle * Ratio); //2006 减速比1:36，6020 1:1  3508 1:19，
    Speed_Ecd = Speed / Ratio;

    if (ID == CAN_YAW_RCV_ID) {
        Angel_Imu = IMU_Angle(1);
        Speed_Imu = IMU_Speed(1);
    } else if (ID == CAN_PIH_RCV_ID) {
        Angel_Imu = IMU_Angle(2);
        Speed_Imu = IMU_Speed(2);
    } else {
        Angel_Imu = IMU_Angle(0);
        Speed_Imu = IMU_Speed(0);
    }

    switch (Which_Mode) {
        case ECD_MODE:
            Angel_All = Angel_Ecd;
            Motor_Speed = Speed_Ecd;
            break;
        case GYR_MODE:
            Angel_All = Angel_Imu;
            Motor_Speed = Speed_Imu;
            break;
    }

    if (timer >= 200) {
        is_online = 0;
    } else {
        is_online = 1;
    }
    // 复位
    if (timer >= 1000) {
        timer = 201;
    }
}

void Motor::CanRcvLoop(CAN_HandleTypeDef *hcan,uint32_t Stdid, uint8_t *recvData) {
    if (hcan == CAN_Line) {
        if (Stdid == ID) //在这里接收数据
        {
            is_online = 1;
            Angel = (int16_t) (recvData[0] << 8 | recvData[1]); // 0~8191
            Speed = (int16_t) (recvData[2] << 8 | recvData[3]); // prm
            Torque = (int16_t) (recvData[4] << 8 | recvData[5]); //转矩电流
            Tempture = (int16_t) (recvData[6]); //温度
            Null = (int16_t) (recvData[7]);
            timer = 0;
            return;
        }
    }
}

void CAN_Filter_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //工作在标识符屏蔽位模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//滤波器位宽为单个32位

    sFilterConfig.FilterIdHigh = 0X0000;
    sFilterConfig.FilterIdLow = 0X0000;
    //过滤屏蔽码
    sFilterConfig.FilterMaskIdHigh = 0X0000;
    sFilterConfig.FilterMaskIdLow = 0X0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    if (hcan->Instance == CAN1)
    {
        sFilterConfig.FilterBank = 0;
    }
    else if (hcan->Instance == CAN2)
    {
        sFilterConfig.FilterBank = 14;
    }
    sFilterConfig.FilterActivation = ENABLE;
    HAL_Status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    if (HAL_Status != HAL_OK)
    {

    }
}

void CAN::All_Init(void) {
    CAN_Filter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_Filter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN::YawSendCurrent(int16_t current) {
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_YAW_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x02;
    send_data[0] = (current >> 8);
    send_data[1] = current & 0xff;

    CAN_TxMessage(&hcan2, &tx_msg, send_data);
}

void CAN::PitchSendCurrent(int16_t current) {
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_PIH_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x04;
    send_data[0] = 0;
    send_data[1] = 0;

    send_data[2] = (current >> 8);
    send_data[3] = current & 0xff;

    CAN_TxMessage(&hcan1, &tx_msg, send_data);
}

void CAN::ChasisSendVal(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_online) {
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_CHASSIS_VAL_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (vx >> 8);
    send_data[1] = vx & 0xff;

    send_data[2] = (vy >> 8);
    send_data[3] = vy & 0xff;

    send_data[4] = (vz >> 8);
    send_data[5] = vz & 0xff;

    send_data[6] = car_mode; //车的模式
    send_data[7] = is_online; //自瞄状态
    CAN_TxMessage(&hcan2, &tx_msg, send_data);
}

void CAN::ChasisSendYaw(int16_t yaw, int16_t pitch, int8_t servo_status, int8_t fric_status, int8_t rammer_status,
                        int8_t is_redrawing) {
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 2;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_CHASSIS_YAW_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (yaw >> 8);
    send_data[1] = yaw & 0xff;

    send_data[2] = (pitch >> 8);
    send_data[3] = pitch & 0xff;

    send_data[4] = servo_status;
    send_data[5] = fric_status;

    send_data[6] = rammer_status;
    send_data[7] = is_redrawing;
    CAN_TxMessage(&hcan2, &tx_msg, send_data);
}

void CAN::ChasisSendCom1(int16_t vx, int16_t vy, int16_t yaw, int8_t car_mode, int8_t is_online) {
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 2;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_CHASIS_COM1_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (vx >> 8);
    send_data[1] = vx & 0xff;

    send_data[2] = (vy >> 8);
    send_data[3] = vy & 0xff;

    send_data[4] = (yaw >> 8);
    send_data[5] = yaw & 0xff;

    send_data[6] = car_mode; //车的模式
    send_data[7] = is_online; //自瞄状态
    CAN_TxMessage(&hcan2, &tx_msg, send_data);
}

void CAN::ShootSendCurrent(int16_t friLc, int16_t friRc, int16_t ramc, int16_t friUc) {
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_SHOOT_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (friLc >> 8);
    send_data[1] = friLc;

    send_data[2] = (friRc >> 8);
    send_data[3] = friRc;

    send_data[4] = (ramc >> 8);
    send_data[5] = ramc;

    send_data[6] = (friUc >> 8);
    send_data[7] = friUc; //步兵就是双枪管用这个

    CAN_TxMessage(&hcan1, &tx_msg, send_data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t recvData[8];
    HAL_StatusTypeDef HAL_Status;
    CAN_RxHeaderTypeDef RxMeg;
    HAL_Status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMeg, recvData);
    if (HAL_Status == HAL_OK) //在这里接收数据
    {
        for (int i = 0; i < 3; ++i) {
            if (i < 2) {
                omni.Get_Motor()[i].CanRcvLoop(hcan,RxMeg.StdId,recvData);
                omni.Get_Motor()[i].timer ++;
            }
            shoot.motors[i].CanRcvLoop(hcan,RxMeg.StdId,recvData);
            shoot.motors[i].timer++;
        }
        if (hcan->Instance == CAN2) {
            if (RxMeg.StdId == CAN_JUDGE_REC_ID) {
                judge.cool_limit = (uint16_t) (recvData[0] << 8 | recvData[1]);
                judge.cool_rate = (uint16_t) (recvData[2] << 8 | recvData[3]);
                judge.heat_now1 = (uint16_t) (recvData[4] << 8 | recvData[5]);
                judge.heat_now2 = (uint16_t) (recvData[6] << 8 | recvData[7]);
                return;
            }
            if (RxMeg.StdId == CAN_JUDGE_SUB_ID) //裁判系统信息的补充
            {
                memcpy(&judge.shoot_spd_now, recvData, 4);
                judge.shoot_spd_max = (uint16_t) ((recvData)[4] << 8 | recvData[5]);
                judge.color = (uint8_t) recvData[6];
                judge.smallOrBig = (uint8_t) recvData[7];
                return;
            }
        }
    }
} //接收回调函数
