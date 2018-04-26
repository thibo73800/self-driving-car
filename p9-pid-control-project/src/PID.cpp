#include "PID.h"
#include <iostream>
#include <math.h>

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
    // To compute the difference between the current CTE and last CTE
    // for the Kd factor
    this->prev_cte = 0;
    // Used to compute the Integral of error for the Ki factor.
    this->int_cte = 0;

    this->throttle = 0.5;

double PID::getNextSteeringAngle(double cte, double speed){
    /*
        This method compute the next steering andle of the car.
        The steering andle is compute using the PID controll method as well
        as the current speed of the car.
        @cte: (Double) value: Current error
        @speed (Double) current speed of the car
    */
    // If this is the first time this method is call...
    if (this->prev_cte == 0)
        prev_cte = cte;

    // If the error is too large, the car go to fast and the error
    // is not being reduced since the last: the car brake
    if (fabs(cte) > 1. && speed > 20 && fabs(cte) > fabs(prev_cte))
        this->throttle = -1;
    else // Otherwie we accelerate to 0.5
        this->throttle = 0.5;

    // I used a factor to control the three others factors (Kp, Ki, Kd)
    // according to the current speed of the car
    double factor = 1.0/log(1.1+(speed/2));
    // Set the error difference since the last error and the current error
    double diff_cte = cte - this->prev_cte;
    this->prev_cte = cte;

    // I don't exactly used the sum of all error from the begining
    // to compute the integral
    this->int_cte = cte + (0.99*this->int_cte);

    // Finnaly, I compute the steering angle
    double steer = -Kp*factor*cte -Kd*factor*diff_cte -Ki*factor*int_cte;

    return steer;
}
