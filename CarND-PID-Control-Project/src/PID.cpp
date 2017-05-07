#include "PID.h"
#include <algorithm>
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

void PID::Init(double aKp, double aKi, double aKd, bool doTwiggle) {
	PID();
	Kp = aKp;
	Ki = aKi;
	Kd = aKd;
	twiggle = doTwiggle;
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

	prev_cte = cte;
	error += cte*cte;
	count++;
	error_count++;

	if(twiggle)
	{
		if(count % 4500 == 0)
		{
			GetBestTwiggleParam(parameterIndex);
			parameterIndex++;
			if(parameterIndex > 2)
				parameterIndex=0;
			PID::TwiggleParameter(GetTotalError(), parameterIndex);
			error = 0;
			error_count=0;
			return;
		}
		if(count % 900 == 0||(error_count > 300 && GetTotalError() > best_err))
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
			twiggleUp = false;
		}
	} else {
		if(error < best_err)
		{
	    	std::cout <<"Twiggle iteration " << count <<" error = " << error << endl;
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
	}

}
