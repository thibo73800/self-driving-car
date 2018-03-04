#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    this->is_initialized = false;
    this->previous_timestamp = 0;

    // measurement covariance matrix
    ekf.R_laser = MatrixXd(2, 2);
    ekf.R_radar = MatrixXd(3, 3);
    // measurement matrix
    ekf.H_laser = MatrixXd(2, 4);
    // State vector (Init later)
    ekf.x = VectorXd(4);
    //state covariance matrix P
	ekf.P = MatrixXd(4, 4);
    // Process covariance matrix (random acceleration vector on the state)
    // Init later
    ekf.Q = MatrixXd(4, 4);
    //the initial transition matrix F
    ekf.F = MatrixXd(4, 4);

    //state covariance matrix P
	ekf.P <<  1, 0, 0,     0,
			  0, 1, 0,     0,
			  0, 0, 1000,  0,
			  0, 0, 0,     1000;
    // the initial transition matrix F
    ekf.F <<  1, 0, 1, 0,
    		  0, 1, 0, 1,
    		  0, 0, 1, 0,
    		  0, 0, 0, 1;
    //measurement matrix
    ekf.H_laser << 1, 0, 0, 0,
    		       0, 1, 0, 0;
    //measurement covariance matrix - laser
    ekf.R_laser <<  0.0225,   0,
                    0,        0.0225;
    //measurement covariance matrix - radar
    ekf.R_radar <<   0.09, 0,        0,
                     0,    0.0009,   0,
                     0,    0,        0.09;
    //measurement matrix
	ekf.H_laser <<  1, 0, 0, 0,
			        0, 1, 0, 0;
    // Process covariance matrix
    ekf.Q << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;

}

// Destructor.
FusionEKF::~FusionEKF() {}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {
    /*
        New measurement. The measurement could come from Lidar Data
        or Radar Data. THis method this method describe the main algorithm =
        parts of the Extended Kalman Filter.
    */
    if (!this->is_initialized) {
        /**
            * Initialize the state ekf.x_ with the first measurement.
            * Create the covariance matrix.
            * Remember: you'll need to convert radar from polar
            * to cartesian coordinates.
        */
        // [First measurement]
        // Laser measurement
        if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
            // If we receive a measurement from the Lidar
            // Add the px and py measurement without any velocity
            ekf.x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0f, 0.0f;
        }
        // Radar measurement
        else{
            ekf.x = Tools::polarToCart(measurement_pack.raw_measurements_);
        }
        this->previous_timestamp = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        this->is_initialized = true;
        return;
    }
    else {
        this->estimation(measurement_pack);
    }
}

void FusionEKF::estimation(const MeasurementPackage &measurement_pack){
    /*
        Estimation function to handle each measurement.
    */
    // Update the timestamp and get the elapsed time from the last measurement.
    float dt = this->updateTime(measurement_pack);
    // Upated the F matrix so that the time is integrated
    ekf.F(0, 2) = dt;
    ekf.F(1, 3) = dt;
    // Update the process covariance matrix (Q)
    this->updateProcessCovariance(dt);
     /*
        [State prediction]
        We start with what we already know to predit the current state of the
        Pedestian.
     */
     ekf.predict();
     /*
        [Measurement update]
        Use new observations to correct our belief about the state of the
        pedestrian/object.
     */
     if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
         // Radar measurements involve a nonlinear measurement function
         // Here, we use the Extended kalman filter equations.
         ekf.updateEKF(measurement_pack.raw_measurements_);
     } else {
         // For Laser measurement, apply a simple Kalman filter
         ekf.update(measurement_pack.raw_measurements_);
     }
     // print the output
     cout << "x_ = " << ekf.x << endl;
     cout << "P_ = " << ekf.P << endl;
}

float FusionEKF::updateProcessCovariance(float dt){
    /*
        Update the process covariance matrix relative to the elapsed time (dt)
    */
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    float dt4 = pow(dt, 4);
    float dt3 = pow(dt, 3);
    float dt2 = pow(dt, 2);

    // First row (px)
    ekf.Q(0, 0) = dt4 * noise_ax / 4;
    ekf.Q(0, 1) = 0;
    ekf.Q(0, 2) = dt3 * noise_ax / 2;
    ekf.Q(0, 3);
    // Second row (py)
    ekf.Q(1, 0) = 0;
    ekf.Q(1, 1) = dt4 * noise_ay / 4;
    ekf.Q(1, 2) = 0;
    ekf.Q(1, 3) = dt3 * noise_ay / 2;
    // Third row (py)
    ekf.Q(2, 0) = dt3 * noise_ax/2;
    ekf.Q(2, 1) = 0;
    ekf.Q(2, 2) = dt2 * noise_ax;
    ekf.Q(2, 3) = 0;
    // Fourth row (py)
    ekf.Q(3, 0) = 0;
    ekf.Q(3, 1) = dt3 * noise_ay / 2;
    ekf.Q(3, 2) = 0;
    ekf.Q(3, 3) = dt2 * noise_ay;

}

float FusionEKF::updateTime(const MeasurementPackage &measurement_pack){
    /*
        Update the last measurement timestamp and return the delta between
        the last measurement and this new measurement.
    */
    float diff = measurement_pack.timestamp_ - this->previous_timestamp;
    float dt =  diff / 1000000.0f;
    this->previous_timestamp = measurement_pack.timestamp_;
    return dt;
}
