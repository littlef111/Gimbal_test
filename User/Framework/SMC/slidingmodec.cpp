#include"slidingmodec.h"
#include<cmath>

//SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5);

void SMC::SMC_Tick(float angle_now,float angle_vel)
{
	angle = angle_now;
    ang_vel = angle_vel;
	float e_qp;
	float qp = q/p;

	error = angle - ref;
	ddref = (ref - refl) - dref;
	dref = (ref - refl);

	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}

	if (error < 0)
		e_qp = -pow(abs(error), qp);
	else
		e_qp = pow(abs(error), qp);

	s = (ang_vel - dref) + C * e_qp;
	ds = -epsilon * Sat(s) - K * s;
	u = J * (ddref + ds - C * qp * (ang_vel - dref) * e_qp / abs(error));

	if (abs(error) < 1)
	{
		s = C * error + (ang_vel - dref);//smc surface
		u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
	}

	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;

	refl = ref;
}