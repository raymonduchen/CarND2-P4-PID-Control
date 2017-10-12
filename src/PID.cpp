#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Speed) {
	// PID Initialization	
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	p_error = 1.0;
	i_error = 1.0;
	d_error = 1.0;

	prev_cte = 0.0;
	it = 0;

	PID::Speed = Speed;

	p.push_back(Kp);
	p.push_back(Ki);
	p.push_back(Kd);
	dp.push_back(1.0);
	dp.push_back(1.0);
	dp.push_back(1.0);

}

void PID::UpdateError(double cte) {
	// PID Update
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
	prev_cte = cte;
	
	it += 1;
}

double PID::TotalError(double Kp, double Ki, double Kd) {
	// Steering value
	double steer = -Kp * p_error - Ki * i_error - Kd * d_error;
	steer = std::max(std::min(1.0, steer), -1.0);

	return steer;
}

