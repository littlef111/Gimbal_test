#include "stm32f4xx_hal.h"
#include "math.h"
#ifndef _SMC_H_
#define _SMC_H_
/*
针对2阶系统
参考链接：
非线性系统（十三）滑模控制解析 - Chenglin Li的文章 - 知乎
https://zhuanlan.zhihu.com/p/138860110
非线性系统（十四）滑模控制器设计流程 - Chenglin Li的文章 - 知乎
https://zhuanlan.zhihu.com/p/139066859
*/

class SMC{
 public:

	float u;
	//初始化列表
	SMC(float C,float K,float ref,float error_eps,uint16_t p,uint16_t q,float u_max,float J,float epsilon):
	C(C),K(K),ref(ref),error_eps(error_eps),p(p),q(q),u_max(u_max),J(J),epsilon(epsilon){};
	//更新函数
	void SMC_Tick(float angle_now,float angle_vel);
	void setC(float value) { C = value; }
	void setK(float value) { K = value; }
	void setRef(float value) { ref = value; }
	void setJ(float value) { J = value; }

	[[nodiscard]] float getC() const { return C; }
	[[nodiscard]] float getK() const { return K; }
	[[nodiscard]] float getRef() const { return ref; }

 private:
	float error;
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面
	float ds;//滑模面的一阶导
	float C;
	float K;
	float ref; //初始目标值
	float error_eps;//误差下限
	uint16_t p; //正奇数
	uint16_t q; //正奇数
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，°/s
	float epsilon;
	// 饱和函数
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}
	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
};
//extern SMC YawSMC;
#endif
