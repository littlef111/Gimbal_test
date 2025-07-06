//
// Created by mjw on 2022/9/19.
//
#include "gimbalc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include <cmath>
#include "shootc.h"
// #include "kalman.h"
#include "iwdgc.h"
#include "packet.hpp"
#include "slidingmodec.h"
#include <complex.h>

#include "usartio.h"

gimbalc omni;
ReceivePacket vision_pkt;
gimbalc::gimbalc()
{
    //matlab pid
    PID_initialize();

    YawBias = ChassisYawTarget;

    Pid_In.PihP_MO = 300;
    Pid_In.PihS_MO = 20192;
    Pid_In.YawP_MO = 300;
    Pid_In.YawS_MO = 30192;
    Pid_In.YawP_LO = -Pid_In.YawP_MO;
    Pid_In.YawS_LO = -Pid_In.YawS_MO;
    Pid_In.PihP_LO = -Pid_In.PihP_MO;
    Pid_In.PihS_LO = -Pid_In.PihS_MO;

    pos_pid[2].setOutStep(5000);
    pos_pid[2].setOutMax(300);
    pos_pid[2].setErrAllMax(30);

    speed_pid[2].setOutMax(30192);
    speed_pid[2].setErrAllMax(7192);
}

void gimbalc::CarChoose(int8_t mode)
{
	switch (mode)
	{
		case TOKEY_MODE:
		{
			YawSMC.setC(28);
			YawSMC.setK(150);

			Pid_In.PihP_P = 1.2;//云台pih轴位置环
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.15;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 600;//pih 300
			Pid_In.PihS_I = 750;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;

			break;
		}

		case GYR_MODE:
		{
			YawSMC.setC(35);
			YawSMC.setK(160);

			Pid_In.PihP_P = 1.5;//云台pih轴位置环
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.12;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 600;//pih 300
			Pid_In.PihS_I = 600;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;
			break;
		}

		case ZIMIAO_MODE:
		{
			YawSMC.setC(16);
			YawSMC.setK(100);

			Pid_In.PihP_P = 0.75;//云台pih轴位置环
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.0;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.03; //前馈一阶差分增益

			Pid_In.PihS_P = 750;//pih 300
			Pid_In.PihS_I = 1000;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;
			break;
		}
	}
}

void gimbalc::AlgorithmCompute()
{

	motors[0].update_angle();
	motors[1].update_angle();

	//普通pid
	ChassisYawPid.Update(motors[0].Angel_Ecd);
	pos_pid[0].Update(motors[0].Angel_All);
	pos_pid[1].Update(motors[1].Angel_All);

	ChassisYawPid.GetOutput();
	pos_pid[0].GetOutput();
	pos_pid[1].GetOutput();

	speed_pid[0].setTarget(pos_pid[0].getOut());
    speed_pid[1].setTarget(pos_pid[1].getOut());

	speed_pid[0].Update(motors[0].Motor_Speed);
	speed_pid[1].Update(motors[1].Motor_Speed);

	speed_pid[0].GetOutput();
	speed_pid[1].GetOutput();

	//滑模控制
	YawSMC.SMC_Tick(motors[0].Angel_All, motors[0].Motor_Speed * 6);

	//Matlab的PID
	Pid_In.YawAngle_Now = motors[0].Angel_All;
	Pid_In.YawSpeed_Now = motors[0].Motor_Speed;
	Pid_In.PihAngle_Now = motors[1].Angel_All;
	Pid_In.PihSpeed_Now = motors[1].Motor_Speed;
	PID_step(1);

	//底盘随动前馈
	ChassisYawPid.setOut(ChassisYawPid.getOut() - Chassis_DifGain * (pos_pid[0].getTarget() - pos_pid[0].getLastTarget()));
    pos_pid[0].setLastTarget(pos_pid[0].getTarget());

}

void gimbalc::Pih_Limit()
{
	if ( motors[1].Which_Mode == ECD_MODE )
	{
		if (PihTarget > Pih_EcdUpLimit) //ecd pitch限位
		{
			PihTarget = Pih_EcdUpLimit;
		}
		if (PihTarget  < Pih_EcdLowLimit)
		{
			PihTarget  = Pih_EcdLowLimit;
		}
	}
	else
	{
		if (PihTarget > Pih_GyrUpLimit) //gyr pitch限位
		{
			PihTarget = Pih_GyrUpLimit;
		}
		if (PihTarget  < Pih_GyrLowLimit)
		{
			PihTarget  = Pih_GyrLowLimit;//-30
		}
	}
}

void gimbalc::Yaw_EcdClean(void) //无问题
{
	motors[0].MotorAngel_ALL = 0;
	motors[0].NowAngel = 0;
	motors[0].Angel_Ecd = motors[0].Angel * 360.0f / (8192.0f);
	ChassisYawPid.setErrAll(0);
} //ecd重置

void gimbalc::Protect_Mode()
{
	if(!motors[0].is_online || !motors[1].is_online || IS_IMU_OK == 0 || !MyRemote.is_online)
	{
		Protect_flag = OFFLINE;
	}
	else
	{
		Protect_flag = ONLINE;
	}

	if (Protect_flag == ONLINE)
	{
		if(Last_Warning != TOKEY && warning == TOKEY)
		{
			fric_ram_status = OPENRAMMER;
			warning = OPENFRIC;
			MyRemote.Last_ProtectMode = OPENFRIC;
		}

		switch (warning)
		{
		case CLOSEFRIC:
		{
			fric_ram_status = CLOSERAMMER;
			shoot.ShootSpeedClean();
			break;
		}

		case OPENFRIC:
		{
			if (Last_Warning != OPENFRIC || Last_ProtectFlag == OFFLINE) //不能在断电的时候由连发切单发，且还是有一定幅度的反转 大概10°
			{
				//rammer电机重置
				shoot.speed_pids[0].setErrAll(0);
                shoot.ram_pos_pid.setTarget(0);
                shoot.ram_pos_pid.setErrAll(0);
				shoot.setrammerflag(0);
				shoot.SetRammer();//通电时默认连发
				shoot.motors[0].clear();
			}
			shoot.ShootSpeedReset();
			fric_ram_status = OPENRAMMER;
			break;
		}
		}
	}
	else if (Protect_flag == OFFLINE)
	{
		Pid_Out.YawCurrent = 0;
		Pid_Out.PihCurrent = 0;
		speed_pid[0].setTarget(0);
        speed_pid[1].setTarget(0);
		YawSMC.u = 0;

		shoot.ShootSpeedClean();
		shoot.RammerSpeedClean();
		shoot.speed_pids[0].setOut(0);

		TargetInit();
		can.YawSendCurrent(0);
		can.PitchSendCurrent(0);

		fric_ram_status = CLOSERAMMER;
		CarMode = SUIDONG;
		Last_CarMode = SUIDONG;
		ChassisYawPid.setErrAll(0);
	}
	Last_Warning = warning;
	Last_ProtectFlag = Protect_flag;
}

void gimbalc::ChassisComLoop()
{
	is_online = 0xff;
	if (Protect_flag == OFFLINE)
	{
		MyRemote.vx = 0;
		MyRemote.vy = 0;
		vz = 0;
		is_online = 0;
	}

	if (MyRemote.rc_ctrl.Get_Key().R.Get_Now_State())fric_ram_status  = CRAZYRAMMER;
	can.ChasisSendCom1(MyRemote.rc_ctrl.Get_RC().ch[2],MyRemote.rc_ctrl.Get_RC().ch[3], -(motors[0].Angel_Ecd - YawBias),MyRemote.rc_ctrl.Get_RC().s[0], is_online);
	switch (CAN2_Status)
	{
	case 0:
		// can.ChasisSendVal(MyRemote.vx*2,MyRemote.vy*2, vz*0.8, 0,is_online ); //错开发送
		CAN2_Status = 1;
		break;
	case 1:
		// can.ChasisSendYaw(-(motors[0].Angel_Ecd - YawBias), 0, 0, shoot.GetFricStatus() & shoot.motors[1].is_online & shoot.motors[2].is_online, fric_ram_status & shoot.getpermit(), MyRemote.portIsRedrawing());//累计误差消除  portIsToSentry()机间通信没用
			CAN2_Status = 0;
		break;
	}
}

void gimbalc::SetWithRC(void)
{
	MyRemote.update();

	CarMode =  MyRemote.CarMode;

	warning = MyRemote.ProtectMode;
	Zimiao = MyRemote.portIsZimiao(); //暫時不改

	vz = -ChassisYawPid.getOut();

	if (Protect_flag == ONLINE) //非保护模式
	{
		if (Zimiao == 0)
		{
			YawTarget += MyRemote.yaw_speed * 5 / 1000;
			PihTarget += MyRemote.pih_speed * 5 / 1000;
			vision_pkt.suggest_fire = 0;
		}
		else
		{
			if (vision_pkt.packat_id != last_ID)//不自瞄的时候 目标值变成NAN了 会不会是当前姿态值有问题
			{
				//在当前姿态值上加减
				lost_timer = 0;
				if(vision_pkt.offset_yaw>284) vision_pkt.offset_yaw -= 360;
				if(vision_pkt.offset_yaw<-284) vision_pkt.offset_yaw += 360;

				if (abs(vision_pkt.offset_pitch - motors[1].Angel_All) > 20 || abs(vision_pkt.offset_yaw - motors[2].Angel_All ) > 75)
				{

				}
				else
				{
					YawTarget = vision_pkt.offset_yaw;
					PihTarget = vision_pkt.offset_pitch;
				}
				last_ID = vision_pkt.packat_id;
			}//自瞄的时候 目标值由上位机给出
			else
			{
				lost_timer++;
			}
			if (lost_timer >= 7)
			{
				vision_pkt.suggest_fire = 0;
				lost_timer = 0;
			}
		}
	}


	if (Zimiao == 0)
	{
		if(MyRemote.Control_Mode == KEY_MODE)CarChoose(TOKEY_MODE);
		else CarChoose(GYR_MODE);
	}
	else CarChoose(ZIMIAO_MODE);

	// if(CarMode == TUOLUO && Zimiao == 1) YawMotorAllAngel.Algorithml = NOMEL;
	if(CarMode == TUOLUO && Zimiao == 1)
	{
//		YawMotorAllAngel.Algorithml = SLIDE;
		YawTarget = vision_pkt.offset_yaw -2.0;
		YawSMC.setJ(0.74);
	}
	else
	{
//		YawMotorAllAngel.Algorithml = SLIDE;
		YawSMC.setJ(0.75);
	}

	switch (CarMode)
	{
	default://随动
	{
		if (Last_CarMode != SUIDONG)
		{
			Yaw_EcdClean();
		}
		if (ChassisYawTarget - motors[0].Angel_Ecd > 180)ChassisYawTarget -= 360; //加减2π
		if (motors[0].Angel_Ecd - ChassisYawTarget > 180)ChassisYawTarget += 360;//优弧劣弧处理
		ChassisYawPid.setTarget(ChassisYawTarget); //YAW_Bias
		break;
	}
	case TUOLUO: //陀螺
	{
		Yaw_EcdClean();
		vz = -50.0f; //恒速小陀螺
		break;
	}
	} //模式切换

	Pih_Limit();
	{
		Pid_In.YawAngle_set = YawTarget;
		Pid_In.PihAngle_set = PihTarget;

		pos_pid[0].setTarget(YawTarget);
		pos_pid[1].setTarget(PihTarget);

		YawSMC.setRef(YawTarget);
	}
	Last_CarMode = CarMode;
	if (MyRemote.rc_ctrl.Get_Key().SHIFT.Get_Now_State() && vz != -ChassisYawPid.getOut()) vz = vz * 2.5; //是否需要分段？ 还是按键定模式
}

void gimbalc::TargetInit(void)
{
	YawTarget = motors[0].Angel_All;
	PihTarget = motors[1].Angel_All; //PIH好像有点搜索不到
}

void gimbalc::CurrentCompute()
{
	switch (motors[0].Algorithml)
	{
	case MATLAB:
		can.YawSendCurrent(motors[0].pole * Pid_Out.YawCurrent);
		break;
	case SLIDE:
		can.YawSendCurrent(motors[0].pole * YawSMC.u);
		break;
	case NOMEL:
		can.YawSendCurrent(motors[0].pole * speed_pid[0].getOut());
		break;
	}

	switch (motors[1].Algorithml)
	{
	case MATLAB:
		can.PitchSendCurrent(motors[1].pole * Pid_Out.PihCurrent);
		break;
	case NOMEL:
		can.PitchSendCurrent(motors[1].pole * speed_pid[1].getOut());
		break;
	}
	can.ShootSendCurrent(shoot.speed_pids[1].getOut(), shoot.speed_pids[2].getOut(), shoot.speed_pids[0].getOut(), 0);
}

//matlab生成控制器 先用一个大循环，后期移植到task
void gimbalc::ControlLoop()
{
	FeedDog(); //喂狗 √
	Printf_Test(); //√
	AlgorithmCompute();
	SetWithRC();
	Protect_Mode();
	ChassisComLoop();
	CurrentCompute();
}

void gimbalc::Printf_Test(void)
{
	// usart_printf("%f\r\n",Pid_Out.PihCurrent);
	// usart_printf("%f\r\n",motors[0].Angel_Ecd);
	// usart_printf("%f,%f,%f,%f,%d,%d\r\n",motors[0].Angel_All,motors[1].Angel_All,motors[0].Motor_Speed,motors[1].Motor_Speed,motors[0].is_online,motors[1].is_online);
} //放一些常用的打印

//以下是改变随动方向的函数接口
int16_t Get_ChassisTarget(void)
{
	return omni.Get_Chassistarget();
}

void Set_ChassisTarget(int16_t Target)
{
	omni.Set_Chassistarget(Target);
}

void Set_YawTarget(float Target)
{
	Target += omni.Get_YawTarget();
	omni.Set_YawTarget(Target);
}