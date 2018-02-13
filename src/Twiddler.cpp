#include "Twiddler.h"
#include <numeric>
#include <limits>
#include <math.h> 
#include <iostream>
#include <sstream>

using namespace std;

Twiddler::Twiddler() {}

Twiddler::~Twiddler() {}

void Twiddler::Init(bool optimize, double Kp_1, double Ki_1, double Kd_1, double Kp_2, double Ki_2, double Kd_2) {
    
    /* 
        Instantiates the Twiddler object and sets the 
    initial parameters for the controllers
    */
    
    optimize_ = optimize;
    restart_simulation_ = false;

    p_ = std::vector<double>(6);
    p_[0] = Kp_1;
    p_[1] = Ki_1;
    p_[2] = Kd_1;
    p_[3] = Kp_2;
    p_[4] = Ki_2;
    p_[5] = Kd_2;
    
    ResetPIDs();
    
    prev_err_ = 0;

    nruns_ = 0;
    step_rate_ = 0.5/100;
    v_ = 0;
    dp_ = 0;

}

double Twiddler::UpdateGradient(double gradErr, double partial) {
    /*
    NOTE :
        If the Twiddler was the main program, the algorithm would begin
    exactly as presented in the lecture notes.     
        But this class was designed to be plugged in an operating environment, so we need to track where we are in the scan loop. N
    */
    
    return gradErr*partial;
}

void Twiddler::UpdateError(double cte, double speed_error){
    /* 
        Calculates the control commands given the errors
    and restarts the simulation in case of saturation of
    commands
    */
    
    // Calculate actuators signals
    pid_steer.UpdateError(cte);
    steer_ = pid_steer.TotalError();
    
    pid_throttle.UpdateError(speed_error);
    throttle_ = pid_throttle.TotalError();
    
    nsteps_++;
    
    err_ += cte + speed_error;
    restart_simulation_ = optimize_ && (fabs(cte) > 2.5);
    
    if (restart_simulation_){
        
        
        /*
        
        p_[0] -= step_rate_*UpdateGradient(-pid_steer.p_cum, err_)/sqrt(nsteps_);
        p_[1] -= step_rate_*UpdateGradient(-pid_steer.i_cum, err_)/sqrt(nsteps_);
        p_[2] -= step_rate_*UpdateGradient(-pid_steer.d_cum, err_)/sqrt(nsteps_);
        
        p_[3] -= step_rate_*UpdateGradient(-pid_throttle.p_cum, err_)/sqrt(nsteps_);
        p_[4] -= step_rate_*UpdateGradient(-pid_throttle.i_cum, err_)/sqrt(nsteps_);
        p_[5] -= step_rate_*UpdateGradient(-pid_throttle.d_cum, err_)/sqrt(nsteps_);
        
        */
        
        /*
        for (v_ = 0; v_ < 6 ; v_ ++){
           p_[v_] = fmin(fmax(p_[v_], 1E-8), 10);
        }*/
        prev_err_ = err_;
        
        ResetPIDs();
        // Terminates run
        nruns_++;
    }
}

bool Twiddler::is_saturated(double command){
    
    return fabs(command) > 1;
}

void Twiddler::ResetPIDs(){
    
    err_ = 0.0;
    nsteps_ = 0;
    
    pid_steer.Init(p_[0], p_[1], p_[2]);
    pid_throttle.Init(p_[3], p_[4], p_[5]);
    
    cout << "PID 1 " << p_[0] << ", " << p_[1] << ", " <<  p_[2] << endl;
    cout << "PID 2 " << p_[3] << ", " << p_[4] << ", " <<  p_[5] << endl;
}