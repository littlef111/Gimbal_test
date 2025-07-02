//
// Created by mjw on 2022/10/7.
//
//shootc 增加模式
#include "shootc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include "math.h"
#include "gimbalc.h"

shootc shoot;

int8_t shootc::GetFricStatus(void)
{
    if (abs(speed_pids[1].getInput()) > 5000 && abs(speed_pids[2].getInput()) > 5000)
    {
        return FRIC_ON;
    }
    else return FRIC_OFF;
}

shootc::shootc()
{
    ram_pos_pid.setComputeType(PositionPID_e);

    speed_pids[0].setWorkType(Ramp_e);
    speed_pids[0].setRampTargetTime(1);
    speed_pids[0].setRampTargetStep(120);
    speed_pids[0].setComputeType(IncrementPID_e);

    speed_pids[1].setWorkType(Normal_e);
    speed_pids[1].setRampTargetTime(3);
    speed_pids[1].setRampTargetStep(100);
    speed_pids[1].setComputeType(IncrementPID_e);

    speed_pids[2].setWorkType(Normal_e);
    speed_pids[2].setRampTargetTime(3);
    speed_pids[2].setRampTargetStep(100);
    speed_pids[2].setComputeType(IncrementPID_e);

    Channel_StopTimeMax = 50;
}

void shootc::FricControl(void)
{
	motors[1].update_angle();
	motors[2].update_angle();

	speed_pids[1].Update(motors[1].Motor_Speed);
	speed_pids[2].Update(motors[2].Motor_Speed);

	speed_pids[1].GetOutput();
	speed_pids[2].GetOutput();
}

void shootc::ShootSpeedClean(void) //clean不用斜坡
{
    speed_pids[1].setTarget(0);
    speed_pids[2].setTarget(0);

    if (abs(speed_pids[1].getInput()) < 1000 || abs(speed_pids[2].getInput()) < 1000) //清除漏电流，防止下载疯转
    {
        speed_pids[1].setOut(0);
        speed_pids[2].setOut(0);
    }
}

void shootc::RammerSpeedClean(void)
{
    speed_pids[0].setWorkType(Normal_e);
    speed_pids[0].setTarget(0);
}

void shootc::ShootSpeedReset()
{
    speed_pids[1].setWorkType(Ramp_e);
    speed_pids[1].setTarget(-SHOOT_SPEED);

    speed_pids[2].setWorkType(Ramp_e);
    speed_pids[2].setTarget(SHOOT_SPEED);
}


void shootc::ShootSpeedTarget(float Ram_Speed, int8_t mode) //bug:有时候目标值弹跳 已解决
{
    ShootSpeedReset();
    if (mode == 1)
    {
        speed_pids[0].setWorkType(Ramp_e);
        speed_pids[0].setTarget(Ram_Speed);
        rammer_flag = 0;
    }
    else if (mode == 2)
    {
        if (rammer_flag == 0) //掉电初始时还是出现大幅度反转
        {
            ram_pos_pid.setErrAll(0);
            ram_pos_pid.setLastInput(0);
            speed_pids[0].setErrAll(0);
            motors[0].clear();
        }
        rammer_flag++;
        ram_pos_pid.setTarget(0 + rammer_flag * 45);
    }
}

void shootc::Stuck_Check(void)
{
    int16_t Current = motors[0].Torque;
    // usart_printf("%d,%d\r\n", Current, stack_time);
    if (Current > 5000) //堵转
    {
        stack_time++;
    }
    if (stack_time >= 100) //堵转超过500ms
    {
        ram_pos_pid.setErrAll(0);
        ram_pos_pid.setErrAll(0);
        ShootSpeedTarget(-20, 1); //卡弹时以100rpm速度反转1s
        reverse_time++;
    }
    if (reverse_time >= reverse_time_max)
    {
        reverse_time = 0;
        stack_time = 0;
    }
}//堵转检测

//计算当前枪口热量
void shootc::Heat_Calcutate()
{
    static uint8_t shootspd_drop = 0;
    if (ram_pos_pid.getInput() > ram_pos_pid.getLastInput() + 40) //开摩擦轮检测到掉速
    {
        shootspd_drop = 1;
        ram_pos_pid.setLastInput(ram_pos_pid.getInput());
    }
    if (shootspd_drop == 1)
    {
        Heat_Cal += 10;  //10是一发小弹丸热量
        shootspd_drop = 0;
    }
    Heat_Cal -= (float) judge.cool_rate / 200.0f;  //周期是5ms
    if (Heat_Cal < 0)
    {
        Heat_Cal = 0;
    }
}

void shootc::SetRammer(void)
{
	// motors[0].update_angle();
	speed_pids[0].Update(motors[0].Motor_Speed);
	speed_pids[0].GetOutput();

	MyRemote.rc_ctrl.Get_Key().Q.portHandle();
	MyRemote.rc_ctrl.Get_Mouse().press_l.portHandle();
	int16_t RammerSpeed = 10;
	Channel_Max = 120.0f; //最大拨弹速度-遥控器

	switch (MyRemote.Control_Mode)
	{
	case KEY_MODE:
		{
			if ( permit && GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.Get_fric_ram_status() == OPENRAMMER)
			{
				speed_pids[0].setWorkType(Ramp_e);
                speed_pids[0].setTarget(ram_pos_pid.getOut());
                if (rammer_flag == 0)
                {
                    speed_pids[0].setWorkType(Ramp_e);
                    speed_pids[0].setTarget(0);
                    ram_pos_pid.setErrAll(0); //积分项清除
                    ram_pos_pid.setTarget(0);
                }

				Channel_Now = MyRemote.rc_ctrl.Get_Mouse().press_l.Get_Now_State();
				if (MyRemote.rc_ctrl.Get_Mouse().press_l.Get_Is_Click_Once())
				{
					ShootSpeedTarget(RammerSpeed, 2);
				}

				motors[0].update_angle();
				ram_pos_pid.Update(motors[0].Angel_All);
				ram_pos_pid.GetOutput();

				if (Channel_Now == Channel_Last && Channel_Now != 0)
				{
					if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常
					{
						RammerSpeed = 50;
						if (MyRemote.rc_ctrl.Get_Key().R.Get_Now_State() == 1) RammerSpeed = 120;//改成与当前热量相关
						ShootSpeedTarget(RammerSpeed, 1);
					}
				}
				else Channel_StopTime = 0;
				Channel_Last = Channel_Now;
			}
			else
			{
				RammerSpeedClean();
				ShootSpeedTarget(0, 1);
			}
			break;
	}
	case RC_MODE:
	{
		if (permit & GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.Get_fric_ram_status() == OPENRAMMER)
		{
			speed_pids[0].setWorkType(Ramp_e);
            speed_pids[0].setTarget(ram_pos_pid.getOut());
            if (rammer_flag == 0)
            {
                speed_pids[0].setWorkType(Ramp_e);
                speed_pids[0].setTarget(0);
                ram_pos_pid.setTarget(0);
                ram_pos_pid.setErrAll(0); //积分项清除
            }

			Channel_Now = abs(MyRemote.rc_ctrl.Get_RC().ch[4] * Channel_Max / 660.0f);

			if (Channel_Now == Channel_Max && Channel_Last != Channel_Max)
			{
				// ShootSpeedTarget(SHOOT_SPEED, -RammerSpeed, 2);
				ShootSpeedTarget(RammerSpeed, 2);
			}

			motors[0].update_angle();
			ram_pos_pid.Update(motors[0].Angel_All);
			ram_pos_pid.GetOutput();

			if (Channel_Now == Channel_Last && Channel_Now != 0)
			{
				if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常 英雄：1000000  步兵：50
				{
					RammerSpeed = Channel_Now;
					ShootSpeedTarget(RammerSpeed, 1);
				}
			}
			else Channel_StopTime = 0;
			Channel_Last = Channel_Now;
		}
		else
		{
			RammerSpeedClean();
			ShootSpeedTarget(0, 1);
		}
		break;
	}
	}
	if(!motors[1].is_online && !motors[2].is_online)
	{
		ram_pos_pid.setTarget(0);
        ram_pos_pid.setErrAll(0);
        speed_pids[0].setErrAll(0);
        rammer_flag = 0;
        motors[0].clear();
	}
}

void shootc::Heat_Protect(void)
{
	Heat_Calcutate();
	if (judge.heat_now1 > Heat_Cal) //默认是Heat_Now1吗
	{
		Heat_Cal = judge.heat_now1 + 20;
	}
	if(judge.heat_now1 < Heat_Cal - 20)
	{
		zerobullettimer++;
		// Heat_Cal = Heat_Now1;
	}
	else zerobullettimer = 0;

	if (zerobullettimer > 200)
	{
		Heat_Cal = judge.heat_now1;
		zerobullettimer = 0;
	}

	if (Heat_Cal > judge.cool_limit - 30)
	{
		permit = FORBID; //即将超热量，不允许发弹
	}
	else permit = PERMIT;
}

void shootc::ControlLoop(void) //发弹主循环：模式切换和速率测试
{
	if (judge.cool_limit == 0)
	{
		judge.cool_limit = 3000;
		judge.cool_rate = 20;
	}
	FricControl();
	Heat_Protect(); //热量超限保护 - 可选？
	SetRammer(); //拨弹轮速度设置
	Stuck_Check(); //卡弹检测
	// usart_printf("%d\r\n",motors[0].Torque);
	// usart_printf("%f,%f,%f,%f,%d\r\n",ram_pos_pid.Target,ram_pos_pid.Input,speed_pids[0].Target,speed_pids[0].Input,MyRemote.is_online);
}
