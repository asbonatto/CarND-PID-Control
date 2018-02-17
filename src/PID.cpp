#include "PID.h"

#include <math.h>
#include <string>
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, string name) {
    // Instantiates a PID controller and sets the error
    // terms to 0
    
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    p_error = 0;
    i_error = 0;
    d_error = 0;
    p_cum = 0;
    i_cum = 0;
    d_cum = 0;
    
    cout << name << " parameters :" << Kp << ", " << Ki << ", " << Kd << std::endl;
    
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    p_cum += p_error;
    i_cum += fabs(cte);
    d_cum += d_error;
    
    
}

double PID::TotalError() {
    // This is actually the command for the actuator
    
    return -(Kp*p_error + Ki*i_error + Kd*d_error);
}
