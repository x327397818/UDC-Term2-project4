#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
	
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
	
	prev_cte = 0.0;
    
    //Twiddle init
    step = 1;
    p.resize(3);
    dp.resize(3);
    
    //make dp feats coefficients
    dp = {0.1 * Kp, 0.1 * Ki, 0.1 * Kd};
    p = {Kp, Ki, Kd};
    
    best_err = std::numeric_limits<double>::max();
    cur_err = 0;
    index = 0;
    pre_index = -1;
    processed = false;
}

void PID::UpdateError(double cte) {
    p_error = cte;
	
	i_error +=cte;
	
	d_error = cte - prev_cte;
	
	prev_cte = cte;
}

double PID::TotalError() {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}

void PID::Twiddle(){
    
    if((dp[0] + dp[1] + dp[2]) > TOLERANCE){
        step++;
        if (step % (BUFF_STEP + EVAL_STEP) > BUFF_STEP){
            cur_err += pow(p_error,2);
        }
    
        if( step % (BUFF_STEP + EVAL_STEP) == 0){
            if ( pre_index != index ){
                if ( cur_err < best_err ){
                    best_err = cur_err;
                }
                p[index] += dp[index];
                pre_index = index;
            } else {
                if ( cur_err < best_err ){
                    best_err = cur_err;
                    dp[index] *= 1.1;
                    index = (index + 1) % p.size();
                    processed = false;
                } else {
                    if( processed == false ){
                        p[index] -= 2*dp[index];
                        processed = true;
                    } else {
                        p[index] += dp[index];
                        dp[index] *= 0.9;
                        index = (index + 1) % p.size();
                        processed = false;
                    }
                }
            }
            cur_err = 0;
            
            // Update class coefficients
            Kp = p[0];
            Ki = p[1];
            Kd = p[2];
        }
    }
}
    
