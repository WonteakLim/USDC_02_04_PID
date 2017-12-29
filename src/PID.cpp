#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    cmd_ = 0.0;

    window_step_ = 0;
    total_error_ = 0.0;

    is_twiddle_ = false;

    window_size_ = 1000;
    
    num_param_ = 3;
    p.clear();
    dp.clear();
    for( int i=0; i<num_param; i++){
	p.push_back(0.0);
	dp.push_back(0.0);
    }    

    update_param_index_ = 0;
    update_subiter_ = 0;
}

void PID::UpdateError(double cte) {
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;

    total_error_ = (window_step_ * total_error_ + pow(cte, 2) ) / (window_step_ + 1);
    window_step_ += 1;

    // Twiddle


    double ctrl_input = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;

    cmd_ = ctrl_input;    
}

double PID::TotalError() {
}

