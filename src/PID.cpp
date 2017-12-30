#include "PID.h"
#include <iostream>

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
    for( int i=0; i<num_param_; i++){
	p.push_back(0.0);
    }
    dp.push_back(0.1*Kp);
    dp.push_back(0.1*Ki);
    dp.push_back(0.1*Kd);    

    update_index_ = 0;
    update_subiter_ = 0;
    iteration_ = 0;

    best_error_ = 1e10;
}

void PID::UpdateError(double cte) {
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;

    double ctrl_input = 0.0;

    if( is_twiddle_ == true){
        total_error_ = (window_step_ * total_error_ + cte*cte ) / (window_step_ + 1);
	window_step_ += 1;

        // Twiddle
	if( window_step_ > window_size_){
	    window_step_ = 0;
	   if( update_index_ >= num_param_ ){
		update_index_ = 0;
		iteration_++;
	    }

	    switch( update_subiter_ ){
		case 0:
		    p[update_index_] += dp[update_index_];
		    update_subiter_ = 1;
		    break;

		case 1:		
		    if( total_error_ < best_error_){
			best_error_ = total_error_;
			dp[update_index_] *= 1.1;
			update_subiter_ = 0;
			update_index_++;
		    }
		    else{
			p[update_index_] -= 2*dp[update_index_];
			update_subiter_ = 2;
		    }
		    break;

		case 2:
		    if( total_error_ < best_error_ ){
			best_error_ = total_error_;
			dp[update_index_] *= 1.1;
		    }
		    else{
			p[update_index_] += dp[update_index_];
			dp[update_index_] *= 0.9;
		    }
		    update_subiter_ = 0;	
		    update_index_++;
		    break;

		default:
		    break;	
	    }

	}

	double Kp = Kp_ + p[0];
	double Ki = Ki_ + p[1];
	double Kd = Kd_ + p[2];

	ctrl_input = -Kp * p_error_ -Ki*i_error_ - Kd*d_error_;

	std::cout << "Window step/error: " << (double)((double)window_step_ / (double)window_size_ ) << " / " << total_error_ << std::endl;
	std::cout << "Twiddle iteration/error: " << iteration_ << "/" << best_error_ << std::endl;
	std::cout << "[Kp, Ki, Kd]: " << Kp << ", " << Ki << ", " << Kd << std::endl;
    }
    else{
	ctrl_input = -Kp_ * p_error_ - Ki_ * i_error_ - Kd_*d_error_;
    }

    cmd_ = ctrl_input;    
}

double PID::TotalError() {
}

