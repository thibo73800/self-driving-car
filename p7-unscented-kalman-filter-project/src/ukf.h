#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {

    public:
        // Constructor & Destructor
        UKF();
        virtual ~UKF();

        // Method used to process each new measurement
        void        ProcessMeasurement(MeasurementPackage meas_package);
        // When the current state is already Init, this method is call
        void        Estimation(MeasurementPackage meas_package);
        // Utils method to compute the time difference
        float       UpdateTime(const MeasurementPackage &measurement_pack);

        // [Prediction Steps]
        void        Prediction(double delta_t);
        // Select augmented sigma points
        MatrixXd    GenerateAugmentedSigmaPoints();
        // Predict the value of each sigma points
        MatrixXd    SigmaPointPrediction(const MatrixXd &Xsig_aug, double dt);
        // Predict a new mean and new covariance matrix using sigma points
        void        PredictMeanAndCovariance(MatrixXd Xsig_pred);

        // [Update from measurement]
        // Translate predicted sigma point to the appropriate space
        void        PredictLaserMeasurement();
        // Update step for the lisar measurements
        void        UpdateLidar(MeasurementPackage meas_package);
        // Translate predicted sigma point to the appropriate space
        void        PredictRadarMeasurement();
        // Update step for the radar measurements
        void        UpdateRadar(MeasurementPackage meas_package);


    public:
        ///* state vector: [px, py, v, φ, φ.] in SI units and rad
        VectorXd x_;

    private:
      ///* initially set to false, set to true in first call of ProcessMeasurement
      bool is_initialized_;
      ///* state covariance matrix
      MatrixXd P_;

      ///* predicted sigma points matrix
      MatrixXd Xsig_pred_;
      // Predicted sigma points in laser/radar space
      MatrixXd Zsig_radar_;
      MatrixXd Zsig_laser_;

      // predicted z in polar space
      VectorXd z_pred_radar_;
      // predicted z in cartesian space
      VectorXd z_pred_laser_;

      // Innovation covariance matrix S for radar/laser
      MatrixXd S_radar_;
      MatrixXd S_laser_;

      ///* Last timestamp
      long long time_us_;

      /*
          Process noise standard deviation longitudinal acceleration in m/s^2
          Process noise standard deviation yaw acceleration in rad/s^2
          The following values have been choose using NSI formula.
      */
      ///* Process noise standard deviation longitudinal acceleration in m/s^2
      double std_a_;
      ///* Process noise standard deviation yaw acceleration in rad/s^2
      double std_yawdd_;

      /*
        Following values are provided by the sensor manufacturer.
      */
      ///* Laser measurement noise standard deviation position1 in m
      double std_laspx_;
      ///* Laser measurement noise standard deviation position2 in m
      double std_laspy_;
      ///* Radar measurement noise standard deviation radius in m
      double std_radr_;
      ///* Radar measurement noise standard deviation angle in rad
      double std_radphi_;
      ///* Radar measurement noise standard deviation radius change in m/s
      double std_radrd_ ;

        /*
            Common variables use for the Unscented Kalman Filter
        */
        // Weights of sigma points
        VectorXd weights_;
        // State dimension
        int n_x_;
        // Augmented state dimension
        int n_aug_;
        // Sigma point spreading parameter
        double lambda_;
};

#endif /* UKF_H */
