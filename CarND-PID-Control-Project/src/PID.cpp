#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():
		prev_cte(0),
		int_cte(0),
		isFirst(true),
		error(0.0),
		count(0),
		error_count(0),
		twiggle(false)
{

}

PID::~PID() {}

void PID::Init(double aKp, double aKi, double aKd, double aKp_t, double aKi_t, double aKd_t, bool doTwiggle) {
	PID();
	Kp = aKp;
	Ki = aKi;
	Kd = aKd;
	Kp_t = aKp_t;
	Ki_t = aKi_t;
	Kd_t = aKd_t;
	best_p[0] = Kp;
	best_p[1] = Ki;
	best_p[2] = Kd;
	best_p[3] = Kp_t;
	best_p[4] = Ki_t;
	best_p[5] = Kd_t;
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	p[3] = Kp_t;
	p[4] = Ki_t;
	p[5] = Kd_t;
	twiggle = doTwiggle;
	parameterIndex=3;
}

void PID::UpdateError(double cte) {

	if(isFirst)
	{
		isFirst=false;
		prev_cte = cte;
		int_cte = cte;
		steering = 0.0;
		return;
	}
	double diff_cte = cte - prev_cte;
	int_cte += cte;
	steering = -Kp * cte - Kd * diff_cte - Ki * int_cte;

//	std::cout << "cte:"<< cte << " diff_cte:" << diff_cte << " int_cte:" << int_cte << " steering: " << steering << endl;
//	std::cout << "-Kp * cte:"<< (-Kp * cte) << " -Kd * diff_cte:" <<  (-Kd * diff_cte) << " - Ki * int_cte:" << (- Ki * int_cte) << " steering: " << steering << endl;

	steering = std::max(steering,-1.0);
	steering = std::min(steering,1.0);

	throttle = -Kp_t * std::abs(cte) - Kd_t * std::abs(diff_cte) - Ki_t * std::abs(int_cte);
	throttle = std::max(throttle,-1.0);
	throttle = std::min(throttle,1.0);

	prev_cte = cte;
	error += cte*cte;
	count++;
	error_count++;

	if(twiggle)
	{
		if(count % 3800 == 0)
		{
			GetBestTwiggleParam(parameterIndex);
			parameterIndex++;
			if(parameterIndex > 5)
				parameterIndex=0;
			PID::TwiggleParameter(GetTotalError(), parameterIndex);
			error = 0;
			error_count=0;
			return;
		}
		if(count % 750 == 0||(error_count > 300 && GetTotalError() > best_err))
		{
			PID::TwiggleParameter(GetTotalError(), parameterIndex);
			error = 0;
			error_count=0;
		}
	}
}

double PID::GetSteering()
{
	return steering;
}

double PID::GetThrottle()
{
	return throttle;
}


double PID::GetTotalError()
{
	return error/error_count;
}

void PID::TwiggleParameter(double error, int paramIndex)
{
	if(twiggleUp)
	{

		if(error < best_err)
		{
			best_err = error;
			dp[paramIndex] *= 1.1;
	    	std::cout <<"Twiggle iteration " << count <<" error = " << error << endl;
	    	best_p[paramIndex] = p[paramIndex];
			p[paramIndex] += dp[paramIndex];

		} else {
			p[paramIndex] -= 2 * dp[paramIndex];
			p[paramIndex] = max(0.00001,p[paramIndex]);
			twiggleUp = false;
		}
	} else {
		if(error < best_err)
		{
	    	std::cout <<"TwigglparamIndexe iteration " << count <<" error = " << error << endl;
			best_err = error;
	    	best_p[paramIndex] = p[paramIndex];
			dp[paramIndex] *= 1.1;
		} else {
			dp[paramIndex] *= 0.9;
		}
		p[paramIndex] += dp[paramIndex];
		twiggleUp = true;

    }
	//set best parameter
	switch(paramIndex)
	{
		case 0:
			Kp = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Kp=" << Kp << endl;
			break;
		case 1:
			Ki = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Ki=" << Ki << endl;
			break;
		case 2:
			Kd = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Kd=" << Kd << endl;
			break;
		case 3:
			Kp_t = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Kp_t=" << Kp_t << endl;
			break;
		case 4:
			Ki_t = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Ki_t=" << Ki_t << endl;
			break;
		case 5:
			Kd_t = p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" tune Kd_t=" << Kd_t << endl;
			break;
	}
}

void PID::GetBestTwiggleParam(int paramIndex)
{
	//set best parameter
	switch(paramIndex)
	{
		case 0:
			Kp = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Kp=" << Kp << endl;
			break;
		case 1:
			Ki = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Ki=" << Ki << endl;
			break;
		case 2:
			Kd = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Kd=" << Kd << endl;
			break;
		case 3:
			Kp_t = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Kp_t=" << Kp_t << endl;
			break;
		case 4:
			Ki_t = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Ki_t=" << Ki_t << endl;
			break;
		case 5:
			Kd_t = best_p[paramIndex];
	    	std::cout <<"Twiggle " << count <<" best Kd_t=" << Kd_t << endl;
			break;
	}

}
