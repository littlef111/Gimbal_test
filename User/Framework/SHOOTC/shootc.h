//
// Created by mjw on 2022/10/7.
//

#ifndef GIMBAL_2023_ABOARD_SHOOTC_H
#define GIMBAL_2023_ABOARD_SHOOTC_H
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "normal_pid.h"
#include "drv_can.h"
#include "gimbalc.h"

class shootc
{
 public:
	Motor motors[3]
	{
		{ECD_MODE,NOMEL,&hcan1,36.0,8192.0f,1,CAN_RAMC_ID},
		{ECD_MODE,NOMEL,&hcan1,1.0,8192.0f,1,CAN_SHOOT_LEFT_ID},
		{ECD_MODE,NOMEL,&hcan1,1.0,8192.0f,1,CAN_SHOOT_RIGHT_ID}
	};
	NormalPID ram_pos_pid{1.5,0,0.05,300};
	NormalPID speed_pids[3]
	{
		{200,15,25,10192},//ram
		{20,3,25,10192},//fricl
		{20,3,25,10192} //fricr
	};
	void ControlLoop();
	void SpeedClean();

	shootc();
	int8_t GetFricStatus(void);
	void RammerSpeedClean(void);
	void ShootSpeedClean(void);
	void ShootSpeedReset();
	void SetRammer(void);

	// getters
	int8_t getrammerflag() { return rammer_flag; }
	int8_t getpermit() { return permit; }

	// setters
	void setrammerflag(int8_t flag) { rammer_flag = flag; }
	void setpermit(int8_t flag) { permit = flag; }

 private:
	int32_t zerobullettimer = 0;
	float Heat_Cal;
	void Heat_Calcutate();
	void Heat_Protect(void);
	void Stuck_Check(void);
	int32_t stack_time = 0;
	int32_t reverse_time = 0;
	void ShootSpeedTarget(float Ram_Speed, int8_t mode);
	void FricControl(void);
	void Protect_Mode();

	int8_t rammer_flag, permit,Protect_flag;//连发慢速还是有点卡
	int16_t reverse_time_max = 80; //80*5ms

	//遥控器相关
	int16_t Channel_StopTime = 0;
	float Channel_Now, Channel_Last, Channel_Max, Channel_StopTimeMax;

	float SHOOT_SPEED = 6200;
};
extern shootc shoot;
#endif //GIMBAL_2023_ABOARD_SHOOTC_H
