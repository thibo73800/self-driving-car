#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::UKF() {
    /**
        Initializes Unscented Kalman filter
        This is scaffolding, do not modify
    */

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    // initial state vector
    x_ = VectorXd(5);
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
        TODO:
        Complete the initialization. See ukf.h for other member properties.
        Hint: one or more values initialized above might be wildly off...
    */

    P_ <<   1, 0, 0,     0, 0,
			0, 1, 0,     0, 0,
			0, 0, 1000,  0, 0,
			0, 0, 0,     1, 0,
            0, 0, 0,     0, 1000;

    /*
        Common variables use for the Unscented Kalman Filter
    */

    // State dimension
    n_x_ = 5;
    // Augmented state dimension
    n_aug_ = 7;
    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;

    weights_ = VectorXd(2 * n_aug_ + 1);
    // Weights of sigma points
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i=1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        weights_(i) =  0.5/ (n_aug_ + lambda_);
    }

    is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
        TODO:
        Complete this function! Make sure you switch between lidar and radar
        measurements.
        * @param {MeasurementPackage} meas_package The latest measurement data of
        * either radar or laser.
    */
    if (!is_initialized_) {
        // [First measurement]
        // Laser measurement
        if (meas_package.sensor_type_ == MeasurementPackage::LASER){
            // If we receive a measurement from the Lidar (A laster measurement)
            // Add the px and py measurement without any velocity
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0f, 0.0f, 0.0f;
        }
        // Radar measurement
        else{
            std::cout << "First measurement from Radar" << '\n';
            x_ = Tools::PolarToCart(meas_package.raw_measurements_);
        }
        time_us_ = meas_package.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
    }
    else{
        this->Estimation(meas_package);
    }
}

float UKF::UpdateTime(const MeasurementPackage &meas_package){
    /*
        Update the last measurement timestamp and return the delta between
        the last measurement and this new measurement.
    */
    float diff = meas_package.timestamp_ - time_us_;
    float dt =  diff / 1000000.0f;
    time_us_ = meas_package.timestamp_;
    return dt;
}

void UKF::Estimation(MeasurementPackage meas_package){
    // Update the timestamp and get the elapsed time from the last measurement.
    float dt = this->UpdateTime(meas_package);
    std::cout << "dt:" << dt << '\n';
    /*
       [State prediction]
       We start with what we already know to predit the current state of the
       Pedestian.
    */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
        std::cout << "Predict" << '\n';
        this->Prediction(dt);
        std::cout << "UpdateRadar" << '\n';
        this->UpdateRadar(meas_package);
    }
}

MatrixXd UKF::GenerateAugmentedSigmaPoints(){
    /*
        Method used to generate Sigma points
    */
    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // [Generate Augmented Sigma Points]

    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0; // I forgot why its zero here ?
    x_aug(6) = 0; // I forgot why its zero here ?

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++) {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    return Xsig_aug;
}

MatrixXd UKF::GenerateSigmaPoints(){
    /*
        Method used to generate Sigma points
    */
    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
    //calculate square root of P
    MatrixXd A = P_.llt().matrixL();

    // [Generate Sigma Points]

    //set first column of sigma point matrix
    Xsig.col(0)  = x_;
    //set remaining sigma points
    for (int i = 0; i < n_x_; i++) {
        Xsig.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
        Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
    }

    return Xsig;
}

MatrixXd UKF::SigmaPointPrediction(const MatrixXd &Xsig_aug, double dt){

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);


    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else {
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
        py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
        v_p = v_p + nu_a*dt;
        yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
        yawd_p = yawd_p + nu_yawdd*dt;

        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }

    return Xsig_pred;
}

void UKF::PredictMeanAndCovariance(MatrixXd Xsig_pred){
    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);
    //create vector for prediced state
    VectorXd x = VectorXd(n_x_);
    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);


    std::cout << "Fill x" << '\n';
    //predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x + weights_(i) * Xsig_pred.col(i);
    }

    std::cout << "Fill P" << '\n';
    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization

        std::cout << "normalization of x_diff(3):" <<  x_diff(3) << '\n';

        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        std::cout << "x_diff(3):" <<  x_diff(3) << '\n';

        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }
    std::cout << "End fill P" << '\n';

    // Assign class values
    x_ = x;
    P_ = P;
}

void UKF::Prediction(double dt) {
    /**
        TODO:
        Complete this function! Estimate the object's location. Modify the state
        vector, x_. Predict sigma points, the state, and the state covariance matrix.
        * Predicts sigma points, the state, and the state covariance matrix.
        * @param {double} dt the change in time (in seconds) between the last
        * measurement and this one.
    */
    std::cout << "GenerateSigmaPoints" << '\n';
    MatrixXd Xsig = this->GenerateSigmaPoints();
    std::cout << "GenerateAugmentedSigmaPoints" << '\n';
    MatrixXd Xsig_aug = this->GenerateAugmentedSigmaPoints();
    std::cout << "SigmaPointPrediction" << '\n';
    MatrixXd Xsig_pred = this->SigmaPointPrediction(Xsig_aug, dt);
    std::cout << "PredictMeanAndCovariance" << '\n';
    this->PredictMeanAndCovariance(Xsig_pred);

    // Assign the value to the class to reuse it later
    Xsig_pred_  = Xsig_pred;

    std::cout << x_ << '\n';
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

void UKF::PredictRadarMeasurement(){

    int n_z = 3;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
    S = S + R;

    S_ = S;
    z_pred_ = z_pred;
    Zsig_ = Zsig;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
        TODO:

        Complete this function! Use radar data to update the belief about the object's
        position. Modify the state vector, x_, and covariance, P_.

        You'll also need to calculate the radar NIS.
    */
    int n_z = 3;

    std::cout << "PredictRadarMeasurement" << '\n';
    this->PredictRadarMeasurement();
    std::cout << "UpdateRadar" << '\n';

    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }

    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
        VectorXd z_diff = Zsig_.col(i) - z_pred_;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_.inverse();
    //residual
    VectorXd z_diff = z - z_pred_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S_*K.transpose();
}
