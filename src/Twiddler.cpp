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
    
    best_err_ = std::numeric_limits<double>::infinity();

    nruns_ = 0;
    step_rate_ = 0.1;
    v_ = 0;
    dp_ = 0.5;
    branch_ = 0;

}

void Twiddler::UpdateGradient() {
    /*
    NOTE :
        If the Twiddler was the main program, the algorithm would begin
    exactly as presented in the lecture notes.     
        But this class was designed to be plugged in an operating environment, so we need to track where we are in the scan loop. N
    */
    
    err_ /= nsteps_; // RMSE
    cout << "Error , best error, dp " << err_ << ", " << best_err_ << ", ";
    if (err_ < best_err_){
        // Accelerate in the direction
        best_err_ = err_;
        dp_ = dp_*(1 + step_rate_);
    }
    else{
        switch (branch_)
        {
        case 0:
            // The first step is to reverse the direction
            dp_ *= -2.0;
            branch_ +=1;
            break;
        case 1:
            // Reduce step in the original direction
            dp_ *= -1.0*(1 - step_rate_);
            branch_ = 0;
            break;    
        }
    }
    cout << dp_ << endl;
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
    
    err_ += pow(cte, 2) + pow(speed_error, 2);
    restart_simulation_ = optimize_ && nsteps_ > 200;
    
    if (restart_simulation_){
        cout << "Restarting simulation" << endl;
        // Terminates run
        nruns_++;
        if(fabs(dp_) < 1E-4){
            v_++;
            dp_ = 0.5;
            cout << "Start optimization of variable # " << v_ + 1 << endl;
            branch_ = 0;
        }
        
        if (v_ > 5){
            std::cout << "Finished optimization" << endl;
        }
        else{
            UpdateGradient();
            p_[v_]+= dp_;
            ResetPIDs();
        }
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
    cout << "PID 2 " << p_[3] << ", " << p_[4] << ", " <<  p_[4] << endl;
}