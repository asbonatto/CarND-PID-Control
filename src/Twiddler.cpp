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
    
    best_err_ = std::numeric_limits<double>::infinity();
    
    v_ = 2;
    ResetTwiddle();
    ResetPIDs();

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
    
    err_ += pow(cte + 0*speed_error, 2);
    restart_simulation_ = optimize_ && (fabs(cte) > 4.5 || nsteps_ > 2500);
    
    double tol = 1E-3;
    if (restart_simulation_){
        
        cout << "# of steps " << nsteps_ << endl;
        err_/= nsteps_; // RMS
        //err_ = -nsteps_;
        
        if (nruns_ == 0){
            best_err_ = err_; // First run for twiddle
            cout << "Setting up error and first search " << endl;
            p_[v_] += dp_;
        }
        else{
            if (nsteps_ > 3){
                if(err_ < best_err_){
                if (direction_ == 0){
                    best_err_ = err_; 
                    dp_*= (1 + step_rate_);
                    p_[v_] += dp_;
                }
                else{
                    dp_*= (1 + step_rate_/2);
                    direction_ = 0;
                }
                
            }
            else{
                cout << "No improvement...";
                
                if (direction_ == 0){
                    cout << " searching other direction" << endl;
                    p_[v_] -= 2.0*dp_;
                    direction_++;
                }
                else{
                    cout << " shrinking step and moving forward again" << endl;
                    p_[v_] += dp_;
                    dp_*= (1 - step_rate_);
                    direction_ = 0;
                    p_[v_] += dp_;
                }
            }
            }
        }
        if (fabs(dp_) < 0.01*p_[v_]) {
            cout << "Finished variable optimization..." << endl;
            v_++;
            v_ = fmod(v_, 3);
            ResetTwiddle();
        }
        else{
            nruns_++;
            if (fabs(p_[v_]) > tol){
                cout << "Current dp (%) " << dp_/p_[v_] << endl;
            }
            else{
                cout << "Current dp " << dp_ << endl;
            }
            
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
    
    pid_steer.Init(p_[0], p_[1], p_[2], "steeering");
    pid_throttle.Init(p_[3], p_[4], p_[5], "throttle");
  
}

void Twiddler::ResetTwiddle(){
    nruns_ = 0;
    step_rate_ = 0.250;
    direction_ = 0;
    
    if (fabs(p_[v_]) > 1E-8){
        dp_ = p_[v_]*0.10;
    }
    else{
        dp_ = 1;
    }
    
}