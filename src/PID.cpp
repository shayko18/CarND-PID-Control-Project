#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	// set all errors to zero
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {
	i_error += cte;         // sum all errors
	d_error = cte-p_error;  // diff current error from the prev error (p_error) 
	p_error = cte;          // current error
}

double PID::TotalError() {
	return Kp*p_error + Ki*i_error + Kd*d_error;  // sum all errors with their coeff
}

