#include "PID.h"

void PID::initialize(float min_val, float max_val, float kp, float ki, float kd)
{
    min_val_ = min_val;
    max_val_ = max_val;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

//位置PID
//入口为 待设定的转速   和 实际的转速
double PID::computeLoc(float setpoint, float measured_value) {
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0) {
        integral_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}
//增量PID
//入口为 待设定的转速   和 实际的转速
//double PID::computeInc(float setpoint, float measured_value){
//    double error;
//    double pid;
//    //setpoint is constrained between min and max to prevent pid from having too much error
//    error = setpoint - measured_value;
//    pid = (kp_ * error) + (ki_ * prev_error_) + (kd_ * prev_error2);
//		prev_error2=prev_error_;
//    prev_error_ = error;
//    return constrain(pid, min_val_, max_val_);
//}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::tostring(char *buf)
{
    sprintf (buf, "[%.4f,%.4f,%4f]",integral_,derivative_,prev_error_);
}