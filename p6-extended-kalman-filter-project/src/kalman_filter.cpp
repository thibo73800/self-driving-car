
#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constructor / Desconstructor
KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::predict() {
    /*
    Predict the next State x
    */

    /*
    Because we know the object position one second ago, we'll assume the
    object kept going at the same velocity. Then, we can predict the next
    state of the obejct.
    X: State/New State
    F: State transition Matrix
    X' = F * X
    mu: Stochastic part, not present in this prediction.
    */
    this->x = this->F * this->x;
    MatrixXd Ft = this->F.transpose();

    /*
    As the object maybe didn't maintin the exact same velocity, when
    we make a prediction, our uncertainty increase.
    P' = F.P.Ft + Q
    P: Uncertainty covariance matrix
    Ft: Transpose of F
    Q: Process covariance matrix (Stochastic part)
    */
    this->P = this->F * this->P * Ft + this->Q;
}

void KalmanFilter::update(const VectorXd &z) {
    /*
    Update the state by using Kalman Filter equations
    */

    /*
    Measurement prediction
    z_hat = H.X
    H: Measurement matrix
    */
    VectorXd z_hat = this->H_laser * this->x;
    /*
    Compute the error between the measurement and the predicted state
    y = z - z_hat
    */
    VectorXd y = z - z_hat;
    // Update from the measurement
    this->_update(y, this->H_laser, this->R_laser);
}

void KalmanFilter::updateEKF(const VectorXd &z) {
    /**
    Update the state by using Extended Kalman Filter equations
    */
    // Compute the Jacobian Matrix Hj
    MatrixXd Hj = Tools::calculateJacobian(this->x);
    // Compute the error between the measurement and the predicted state
    // y = z - h(x)
    VectorXd y = z - Tools::cartToPolar(this->x);
    // Normalize the angle phi to range -pi to pi.
    while (y[1] < -M_PI) {
        y[1] += 2.0 * M_PI;
    }
    while (y[1] > M_PI) {
        y[1] -= 2.0 * M_PI;
    }
    // Update from the measurement
    this->_update(y, Hj, this->R_radar);
}

void KalmanFilter::_update(const VectorXd y, const MatrixXd &H, const MatrixXd R){
    /*
        One part of the Kalman algorithm do not change and can be common to
        both Sensor (Laser and Radar).
        y: the measurement/prediction error
        H: Measurement matrix to use
        R: Measurement covariance matrix
    */
    /*
    S = H.P.Ht.R
    H: Measurement matrix
    R: Measurement Noise covariance matrix
    P: Uncertainty covariance matrix
    */
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * this->P * Ht + R;

    /*
    Kalman filter Gain. Combines the uncertainty of where we think we are
    and the true measurement. The goal is to give more weight either to the
    measurement or where we think we are using a simple prediction.
    (Compare R and P)
    */
    MatrixXd K = this->P * Ht * S.inverse();

    // New estimation
    this->x = this->x + (K * y);
    MatrixXd I = MatrixXd::Identity(this->x.size(), this->x.size());
    this->P = (I - K * H) * this->P;
}
