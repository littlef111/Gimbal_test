//
// Created by LENOVO on 2022/9/6.
//

#ifndef ROBO_TEST_CORE_INC_BSP_CAN_H_
#define ROBO_TEST_CORE_INC_BSP_CAN_H_

#include "stm32f4xx_hal.h"
#include "can.h"
//pitch can1
//yaw can2
typedef enum
{
	CAN_YAW_RCV_ID = 0x209,
	CAN_PIH_RCV_ID = 0x206,//0x204+n

	CAN_YAW_SEND_ID = 0x2FE,
	CAN_PIH_SEND_ID = 0x1FF,

	CAN_CHASSIS_VAL_ID = 0x401,
	CAN_CHASSIS_YAW_ID = 0x402,

	CAN_SHOOT_LEFT_ID = 0x201,
	CAN_SHOOT_RIGHT_ID = 0x202,
	CAN_RAMC_ID = 0X203,

	CAN_SHOOT_SEND_ID = 0x200,

	CAN_JUDGE_REC_ID = 0x405,
	CAN_JUDGE_SUB_ID = 0x407,

	CAN_CHASIS_COM1_ID = 0x107,
	CAN_CHASIS_COM2_ID = 0x108,
} CAN_Msg_enum;

class Motor
{
 public:
	//检测是否掉线
	uint16_t is_online = 0;
	uint32_t timer = 0;

	//实际控制时用到的变量
	double Angel_All;
	double Motor_Speed;

	double Angel_Ecd;
	double Speed_Ecd;
	double Angel_Imu;
	double Speed_Imu;

	//can反馈获得的数据
	int16_t Angel;
	int16_t Speed;
	int16_t Torque;
	uint8_t Tempture;
	uint8_t Null;

	//CAN反馈时，连续化用到的变量
	int64_t MotorAngel_ALL;
	uint16_t NowAngel;
	uint16_t LastAngel;
	int16_t IncreAngel;

	uint8_t Which_Mode; //反馈模式
	uint8_t Algorithml; //控制算法
	CAN_HandleTypeDef *CAN_Line = &hcan1; //CAN1 还是 CAN2
	float Ratio = 19.0f; //减速比
	float EncoderPerCircle = 8192.0f; //单圈脉冲数
	int8_t pole = 1; //极性
	uint16_t ID = 0x000;

	Motor(uint8_t Which_Mode, uint8_t Algorithml, CAN_HandleTypeDef *CAN_Line,float Ratio ,float EncoderPerCircle, int8_t pole,uint16_t ID):Which_Mode(Which_Mode),Algorithml(Algorithml),CAN_Line(CAN_Line),Ratio(Ratio),EncoderPerCircle(EncoderPerCircle),pole(pole),ID(ID){};
	void CanRcvLoop(CAN_HandleTypeDef *hcan,uint32_t Stdid, uint8_t *recvData);
	void update_angle();
	void clear();
//	int *send();
 private:
	const int16_t ContinueAngelMax = 5000; //角度连续化比较角度
	int16_t GetEncoderContinueAngel()
	{ // 角度连续化，并判断方向
		int16_t Incre = 0;
		Incre = NowAngel - LastAngel;
		if (Incre >= 0 && Incre < ContinueAngelMax)
		{ //正转
			Incre += 0;
		}
		if (Incre < -ContinueAngelMax)
		{ //正转，并且不连续
			Incre += 8192;
		}
		if (Incre <= 0 && Incre > -ContinueAngelMax)
		{ //反转
			Incre += 0;
		}
		if (Incre > ContinueAngelMax)
		{ //反转，并且不连续
			Incre -= 8192;
		}
		return Incre;
	}
};
class JudgeMessage
{
 public:
	uint16_t heat_now1, heat_now2, cool_limit, cool_rate, shoot_spd_max;
	float shoot_spd_now = 0;
	uint8_t color, smallOrBig;
};

class CAN
{
 public:
	void All_Init();
	void YawSendCurrent(int16_t current);
	void PitchSendCurrent(int16_t current);
	void ChasisSendVal(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_online);
	void ChasisSendYaw(int16_t yaw, int16_t pitch,int8_t servo_status,int8_t fric_status,int8_t rammer_status ,int8_t is_redrawing);
	void ShootSendCurrent(int16_t friLc, int16_t friRc, int16_t ramc, int16_t friUc);\
	void ChasisSendCom1(int16_t vx, int16_t vy, int16_t yaw, int8_t car_mode, int8_t is_online);
 private:
	// void Filter_Init(CAN_HandleTypeDef* hcan);

};

extern CAN can;
extern JudgeMessage judge;
#endif //ROBO_TEST_CORE_INC_BSP_CAN_H_