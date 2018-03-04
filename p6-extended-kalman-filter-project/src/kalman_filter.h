#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
    public:
        // Constructor && Desconstructor
        KalmanFilter();
        virtual ~KalmanFilter();

        // Predict the next state
        void predict();
        // Update the state for Laser measurements
        void update(const Eigen::VectorXd &z);
        // Update the state for Ladar measurements
        void updateEKF(const Eigen::VectorXd &z);

        // Current state (px, py, vx, vy)
        Eigen::VectorXd x;
        // Uncertainty covariance matrix
        Eigen::MatrixXd P;
        // Transition matrix
        Eigen::MatrixXd F;

        // Process covariance matrix
        Eigen::MatrixXd Q;
        // Measurement matrix for the laser
        Eigen::MatrixXd H_laser;
        // Measurement covariance matrix
        Eigen::MatrixXd R_laser;
        Eigen::MatrixXd R_radar;

    private:
        // Upate the state and the uncertainty cova-matrix (For Laser and Radar)
        void _update(const Eigen::VectorXd y, const Eigen::MatrixXd &H, const Eigen::MatrixXd R);
};

#endif /* _KALMAN_FILTER_H_ */
