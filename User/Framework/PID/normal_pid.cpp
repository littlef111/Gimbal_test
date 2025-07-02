//
// Created by mjw on 2022/9/1.
//
#include "normal_pid.h"

void NormalPID::Update(float NowInput)
{
	Input = NowInput;
	if (State_RampOrNormal == Ramp_e)
	{
		if (RampCountTime < RampTargetTime)
		{
			++RampCountTime;
		}
		else
		{
			RampCountTime = 0;
			if (RealTarget < Target)
			{ //斜坡函数还没计数完，
				RealTarget += RampTargetStep;
				if (RealTarget >= Target)
				{
					RealTarget = Target;
					State_RampOrNormal = Normal_e;
				}
			}
			else if (RealTarget > Target)
			{
				RealTarget -= RampTargetStep;
				if (RealTarget <= Target)
				{
					RealTarget = Target;
					State_RampOrNormal = Normal_e;
				}
			}
			else
			{ //斜坡函数计数已经完成，要退出斜坡函数模式
				State_RampOrNormal = Normal_e;
			}
		}
	}
	else if (State_RampOrNormal == Normal_e)
	{
		RealTarget = Target;
	}

	Err_lastlast = Err_last;
	Err_last = Err_now;
	Err_now = RealTarget - NowInput;

	if (Err_now < Precision && Err_now > -Precision)
		Err_now = 0;

	Err_all += Err_now;

	if (Err_all > ErrAllMax)
	{
		Err_all = ErrAllMax;
	}
	else if (Err_all < -ErrAllMax)
	{
		Err_all = -ErrAllMax;
	}
}

void NormalPID::GetOutput()
{
	if (ComputeType == PositionPID_e)
	{
		Out = Kp * Err_now + Kd * (Err_now - Err_last);
		Out += (Err_all * Ki);
	}
	else if (ComputeType == IncrementPID_e)
	{
		Out += Ki * Err_now + Kp * (Err_now - Err_last);
		Out += Kd * (Err_now - 2 * Err_last + Err_lastlast);
	}
	if (Out >= OutMax)
		Out = OutMax;
	if (Out <= -OutMax)
		Out = -OutMax;
	
	if (Out - lastout > OutStep)
	{
		Out = lastout + OutStep;
	}
	if (Out - lastout < -OutStep)
	{
		Out = lastout + -OutStep;
	}
	
	lastout = Out;
}

void NormalPID::Clear()
{
	Err_now = 0.0;
	Err_last = 0.0;
	Err_lastlast = 0.0;
	Err_all = 0.0;
	Out = 0.0;
	lastout = 0.0;
	RealTarget = 0.0;
	Input = 0.0;
	State_RampOrNormal = Normal_e;
	Target = 0.0;
	RampCountTime = 0.0;
}
