//
// Created by mjw on 2022/9/19.
//

#ifndef ROBO_TEST_GIMBALC_H
#define ROBO_TEST_GIMBALC_H
#include "packet.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "PIDC.h"
#include "drv_can.h"
#include "slidingmodec.h"
#include "normal_pid.h"
////兵种选择
//#define MECANUM1 1 //老麦轮 √ 已退休
//#define MECANUM2 2 //新麦轮 √
//#define OMNI 3 //全向轮
//#define HERO 4 //英雄云台 √
//#define BALANCE 5//平衡步兵
//
//#define GIMBAL OMNI

//电机反馈选择
#define ECD_MODE 0
#define GYR_MODE 1
#define ZIMIAO_MODE 2
#define TOKEY_MODE 3

//选择是哪个角
#define YAW_ANGLE 1
#define PIH_ANGLE 2
#define RAM_ANGLE 3

//选择操作模式
#define KEY_MODE 1
#define RC_MODE 2

//算法选择
#define NOMEL 1
#define MATLAB 2
#define SLIDE 3

//模式赋值
// #define ZIYOU  1
#define SUIDONG  3
#define TUOLUO  2
#define SPIN 4
#define CAR_PROTECT 5
#define RC_OFFLINE 6

//保护模式赋值
#define PROTECT 4
#define TOKEY 2
#define OPENFRIC 3
#define CLOSEFRIC 1
#define OFFLINE 0
#define ONLINE 1

//摩擦轮状态
#define CLOSERAMMER 0
#define OPENRAMMER 1
#define CRAZYRAMMER 2

//舵机状态
#define SERVO_OFF 0
#define SERVO_ON 1

//是否允许发弹
#define FORBID 0
#define PERMIT 1

//实际摩擦轮状态
#define FRIC_OFF 0
#define FRIC_ON 1

extern ExtU rtU;
extern ExtY rtY;
extern ReceivePacket vision_pkt;

#define Pid_In rtU
#define Pid_Out rtY //matlab生成的PID

int16_t Get_ChassisTarget(void);
void Set_ChassisTarget(int16_t Target);
void Set_YawTarget(float Target);

class gimbalc
{
private:
	float Chassis_DifGain = 15, ChassisYawTarget = 104.0f, YawBias, vz;
	float Pih_EcdUpLimit = 173, Pih_EcdLowLimit = 125, Pih_GyrUpLimit = 30, Pih_GyrLowLimit = -28;
	float YawTarget,PihTarget;
	int8_t fric_ram_status;

	Motor motors[2]
	{
		{GYR_MODE,SLIDE,&hcan2,1.0,8192.0f,1,CAN_YAW_RCV_ID},
		{GYR_MODE,MATLAB,&hcan1,1.0,8192.0f,-1,CAN_PIH_RCV_ID},
	};
//	NormalPID
	NormalPID pos_pid[2]
	{
		{1,1,1,1},
		{1,1,1,1}
	};
	NormalPID speed_pid[2]
	{
		{1,1,1,1},
		{1,1,1,1}
	};
	NormalPID ChassisYawPid{0.8,0.0,0.32,1000};
// 	SMC
	SMC YawSMC{20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5};

	bool CAN2_Status;
	int8_t warning,Last_Warning,CarMode,Last_CarMode,is_online,Protect_flag,Last_ProtectFlag,Zimiao;
	int8_t last_ID,lost_timer;

	void Protect_Mode();
	void CarChoose(int8_t mode);
	void ChassisComLoop();
	void AlgorithmCompute();
	void SetWithRC(void);
	void TargetInit(void);
	void CurrentCompute();
	void Pih_Limit();
	void Yaw_EcdClean(void);

public:
	gimbalc();
	void Printf_Test(void);
	void ControlLoop(void);
	int8_t Get_fric_ram_status() { return fric_ram_status; };
	Motor *Get_Motor() { return motors; };
	int16_t Get_Chassistarget() { return ChassisYawTarget; };
	void Set_Chassistarget(int16_t Target) {ChassisYawTarget = Target; };
	float Get_YawTarget() { return YawTarget; };
	void Set_YawTarget(float Target) {YawTarget = Target; };
};
extern gimbalc omni;
#ifdef __cplusplus
}
#endif
#endif //ROBO_TEST_GIMBALC_H
