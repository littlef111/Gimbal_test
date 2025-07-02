//
// Created by mjw on 2022/9/1.
//

#ifndef ROBO_TEST_CORE_INC_PID_H_
#define ROBO_TEST_CORE_INC_PID_H_

#include "stm32f4xx_hal.h"
#define PID_DEFAULT_PRECISION 0.0f		 //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 3000		 //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 10192	 //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192 //输出微分限幅

typedef enum
{
	IncrementPID_e = 0, //增量式PID
	PositionPID_e = 1,  //位置式PID
} PID_Type_e;

typedef enum
{
	Normal_e = 0,			//PID工作在正常状态
	Ramp_e = 1,				//ID工作在斜坡函数状态
	Ramping_Add = 2,		//斜坡还没完成，并且是加
	Ramping_Reduce = 3,	  	//斜坡还没完成，并且是减

	RampDefault_Step = 2, 	//斜坡函数步幅
	RampDefault_Time = 1,	//计数多少次才把count加上1个步幅
} PID_RampState_e;

class NormalPID
{
 public:
	void Update(float NowInput);
	void GetOutput();
	void Clear();
	NormalPID(float kp,float ki,float kd,float OutMax):Kp(kp),Ki(ki),Kd(kd),OutMax(OutMax){};

	// Getter functions
    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }
    float getErrNow() const { return Err_now; }
    float getErrLast() const { return Err_last; }
    float getErrAll() const { return Err_all; }
    float getOut() const { return Out; }
    float getTarget() const { return Target; }
    float getInput() const { return Input; }
    float getLastInput() const { return LastInput; }
    uint8_t getWorkType() const { return WorkType; }
    uint8_t getComputeType() const { return ComputeType; }
    float getPrecision() const { return Precision; }
    float getErrAllMax() const { return ErrAllMax; }
    float getOutMax() const { return OutMax; }
    float getOutStep() const { return OutStep; }
	float getLastTarget() const { return lastTarget; }

    // Setter functions
    void setKp(float kp) { Kp = kp; }
    void setKi(float ki) { Ki = ki; }
    void setKd(float kd) { Kd = kd; }
    void setErrNow(float err) { Err_now = err; }
    void setErrLast(float err) { Err_last = err; }
    void setErrAll(float err) { Err_all = err; }
    void setOut(float out) { Out = out; }
    void setTarget(float target) { Target = target; }
    void setInput(float input) { Input = input; }
    void setLastInput(float input) { LastInput = input; }
    void setWorkType(uint8_t type) { WorkType = type; }
    void setComputeType(uint8_t type) { ComputeType = type; }
    void setPrecision(float precision) { Precision = precision; }
    void setErrAllMax(float max) { ErrAllMax = max; }
    void setOutMax(float max) { OutMax = max; }
    void setOutStep(float step) { OutStep = step; }
	void setRampTargetTime(uint8_t time) { RampTargetTime = time; }
	void setRampTargetStep(float step) { RampTargetStep = step; }
	void setLastTarget(float target) { lastTarget = target; }

 private:
	float Kp;
	float Ki;
	float Kd;

	float Err_now = 0;
	float Err_last = 0;
	float Err_lastlast = 0;
	float Err_all = 0;

	float Out = 0;
	float lastout = 0;

	float Target = 0;
	float RealTarget = 0;
	float lastTarget = 0;

	float Input = 0;
	float LastInput = 0;

	uint8_t WorkType = Normal_e;       //斜坡还是阶跃
	uint8_t State_RampOrNormal = Normal_e;
	uint8_t RampCountTime = 0;
	uint8_t RampTargetTime = RampDefault_Time; //计数多少次才把count加上1个步幅
	float RampTargetStep = RampDefault_Step;   //斜坡函数步幅

	uint8_t ComputeType = PositionPID_e;       // PID工作在位置式还是增量式
	uint8_t Connected = 1;

	float Precision = PID_DEFAULT_PRECISION; // PID最小精度
	float ErrAllMax = PID_DEFAULT_ERRALL_MAX; // PID积分限幅
	float OutMax = PID_DEFAULT_OUTPUT_MAX;	 // PID输出限幅
	float OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;	 // PID输出步幅限制

};
//第一步，初始化列表
//NormalPID XXX(1,1,1,1000);
//第二步，设置目标值更新方式和计算方式
//XXX.WorkType = Normal_e;
//XXX.ComputeType = PositionPID_e;
//第三步，得到输出
//XXX.GetOutput();
//第四步，使用can发送函数输出pid计算结果
//YourCanSendFunction(XXX.Out);

#endif //ROBO_TEST_CORE_INC_PID_H_
