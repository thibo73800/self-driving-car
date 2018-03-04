#include "tools.h"
#include <cmath>
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

// Constructor && Desconstructor
Tools::Tools() {}
Tools::~Tools() {}

VectorXd Tools::calculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
        Method to calculate the RMSE
    */
    VectorXd rmse(4);
  	rmse << 0,0,0,0;

  	/*
        Check the validity of the following inputs:
  	         - The estimation vector size should not be zero
  	         - The estimation vector size should equal ground truth vector size
    */
  	if (estimations.size() != ground_truth.size() || estimations.size() == 0){
  		cout << "Invalid estimation or ground_truth data" << endl;
  		return rmse;
  	}
  	// Accumulate squared residuals
  	for(unsigned int i=0; i < estimations.size(); ++i){
  		VectorXd residual = estimations[i] - ground_truth[i];
  		//coefficient-wise multiplication
  		residual = residual.array()*residual.array();
  		rmse += residual;
  	}

  	//calculate the mean
  	rmse = rmse/estimations.size();
  	//calculate the squared root
  	rmse = rmse.array().sqrt();
  	//return the result
  	return rmse;
}

MatrixXd Tools::calculateJacobian(const VectorXd &x_state) {
    /*
        Method used to calculateJacobian Matrix Hj. We compute the
        Jacobian in order to apply the Extented Kalman filter.
        The Jacobian matrix allow to use a linear approximation of h(x) and
        to work with a gaussian distribution though..
    */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Denominator
    float add_xy2 = pow(px, 2) + pow(py, 2);
    float sqrt_axy2 = sqrt(add_xy2);
    float exp_xy2 = pow(add_xy2, 3.0f/2.0f);

    Hj <<   0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    // Too close to zero...
    if (add_xy2 < 0.000001f){
        return Hj;
    }
    else {
        // Create the jacobian matrix
        Hj << px/sqrt_axy2, py/sqrt_axy2,  0, 0,
  	 	     -py/add_xy2,   px/add_xy2,    0, 0,
  	         (py*(vx*py-vy*px))/exp_xy2, (px*(vy*px-vx*py))/exp_xy2, px/sqrt_axy2, py/sqrt_axy2;
    }
	return Hj;
}

VectorXd Tools::cartToPolar(const VectorXd &state){
    /*
        Method used to translate cartesian coordinate (px, py, vx, vy)
        to polar coordinate (p, tho, p_dot)
    */
    VectorXd polar = VectorXd(3);

    float px = state[0];
    float py = state[1];
    float vx = state[2];
    float vy = state[3];

    float p = sqrt(pow(px, 2) + pow(py, 2));
    float phi = atan2(py, px);
    float p_dot = ((px*vx)+(py*vy)) / p;

    polar << p, phi, p_dot;
    return polar;
}

VectorXd Tools::polarToCart(const VectorXd &state) {
    /*
        Method used to translate polar coordinate (p, tho, p_dot) to
        cartesian coordinate (px, py, vx, vy);
    */
    VectorXd cart = VectorXd(4);

    const float p = state[0];
    const float phi = state[1];
    const float p_dot = state[2];

    // Trigonometri to decude the position of px and py
    const float px = p * cos(phi);
    const float py = p * sin(phi);

    cart << px, py, 0.0, 0.0;
    return cart;
}
