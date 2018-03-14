#include "ukf.h"
#include "Eigen/Dense"
#include <fstream>
#include <sstream>
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::~UKF() {}
UKF::UKF() {
    /**
        Initializes Unscented Kalman filter
    */

    /*
        initial state vector
        [px, py, v, φ, φ.]
    */
    x_ = VectorXd(5);
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    P_ <<   0.05,   0,      0,  0,      0,
			0,      0.02,   0,  0,      0,
			0,      0,      1., 0,      0,
			0,      0,      0,  0.01,   0,
            0,      0,      0,  0,      1.;

    /*
        Process noise standard deviation longitudinal acceleration in m/s^2
        Process noise standard deviation yaw acceleration in rad/s^2
        The following values have been choose using NSI formula.
    */
    std_a_ = 2.8;
    std_yawdd_ = 0.8;

    /*
        DO NOT MODIFY measurement noise values below these are provided by the
        sensor manufacturer.
    */
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

    /*
        The following variables are used to implement the Unscented Kalman
        Filter algorithm in the class methods.
    */
    n_x_ = 5; // State dimension
    n_aug_ = 7; // Augmented state dimension
    lambda_ = 3 - n_x_; // Sigma point spreading parameter
    is_initialized_ = false; // For the fist measurement

    /*
        Weights explain ...
    */
    weights_ = VectorXd(2 * n_aug_ + 1);
    // Weights of sigma points
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i=1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        weights_(i) =  0.5/ (n_aug_ + lambda_);
    }
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
        @param {MeasurementPackage} meas_package The latest measurement data
        of either radar or laser.
    */
    if (!is_initialized_) {
        // [First measurement]
        Tools::emptyLogFiles();
        // Laser measurement
        if (meas_package.sensor_type_ == MeasurementPackage::LASER){
            // If we receive a measurement from the Lidar (A laser measurement)
            // Add the px and py measurement without any velocity
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0f, 0.0f, 0.0f;
        }
        // Radar measurement
        else{
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
    /*
       [State prediction]
       We start with what we already know to predit the current state of the
       Pedestian.
    */
    this->Prediction(dt);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
        this->UpdateRadar(meas_package);
    }
    else{
        this->UpdateLidar(meas_package);
    }
}

MatrixXd UKF::GenerateAugmentedSigmaPoints(){
    /*
        To deal with nonlinear function (The integral gives a result which
        is not normally distributed anymore), UKF uses the
        Unscented Transformation. This aim is to compute an approximation of a
        new mean vector and covariance matrix. To process the transformation
        UKF uses Sigma point. This method generate the appropriate sigma points.
    */

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // [Generate Augmented Sigma Points]

    /*
        The sigma points are chosen around the mean state and in a certain
        relation to the standard deviation of every state dimension.

        X(k|k) = [x(k|k)    x(k|k)+sqrt((λ+nx).P(k|k))  x(k|k)-sqrt((λ+nx).P(k|k))]
        Here we simply assign the first column of the matrix: x(k|k)
        We expend the dimension because we use an augmentation of the state
        to handle the process noise vector.
    */
    x_aug.head(5) = x_;
    x_aug(5) = 0; // TODO Test with other values here
    x_aug(6) = 0;

    /*
        We use the augmented covariance matrix
        P(a,k|k) = [
                P(k|k)  0
                0       Q*
        ]
    */
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++) {
        /*
            X(k|k) = [x(k|k)    x(k|k)+sqrt((λ+nx).P(k|k))  x(k|k)-sqrt((λ+nx).P(k|k))]
            Here we assign the two others type of columns:
                x(k|k)+sqrt((λ+nx).P(k|k))
                x(k|k)-sqrt((λ+nx).P(k|k))
        */
        Xsig_aug.col(i+1)           = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_)    = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    return Xsig_aug;
}

MatrixXd UKF::SigmaPointPrediction(const MatrixXd &Xsig_aug, double dt){
    /*
        Sigma Point Prediction using the integral formula
        (CTRV Differential Equation)
        x(k+1) = x(k) + I(k+1, k)[x(k)]dt + ν
        v: noise vector
    */
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

        /*
            x(k+1) = x(k) + I(k+1, k)[x(k)]dt + ν
            Compute this part:
                x(k) + I(k+1, k)[x(k)]dt
            We also prevent any division by zero
        */
        if (fabs(yawd) > 0.001) { // Normal integral
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else { // The object follow a straight line
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }
        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;

        /*
            x(k+1) = x(k) + I(k+1, k)[x(k)]dt + ν
            Add the stochastic part of the process model
            The process nosie vector ν
        */
        px_p =      px_p    + 0.5*nu_a*dt*dt * cos(yaw);
        py_p =      py_p    + 0.5*nu_a*dt*dt * sin(yaw);
        v_p =       v_p     + nu_a*dt;
        yaw_p =     yaw_p   + 0.5*nu_yawdd*dt*dt;
        yawd_p =    yawd_p  + nu_yawdd*dt;

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
    /*
        We have pick our sigma points, predict the value of each sigma points.
        Now, it's time to deduce an approximation of the mean and the covariance
        matrix of these sigma points.

        Predicted the mean:
            x(k+1|k) = Σ(no, i=0) w(i)X(k+1|k,i)
        Predicted covariance:
            P(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(X(k+1|k,i) - x(k+1|k))T

        The weights invert the spreading of the sigma points
        The weights are init in the Constructor of this class with the following
        values:
            w(i) = λ/(λ+na),        i=0
            w(i) = 1/(2*(λ+na)),    i=2....na

    */
    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);
    //create vector for prediced state
    VectorXd x = VectorXd(n_x_);
    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);

    /*
        Predict the state mean vector
        x(k+1|k) = Σ(no, i=0) w(i)X(k+1|k,i)
    */
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x + weights_(i) * Xsig_pred.col(i);
    }

    /*
        Predict the new covariance matrix
        P(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(X(k+1|k,i) - x(k+1|k))T
    */
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference: (X(k+1|k,i) - x(k+1|k))
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        // w(i) . (X(k+1|k,i) - x(k+1|k)) . (X(k+1|k,i) - x(k+1|k))T
        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }

    // Assign class values
    x_ = x;
    P_ = P;
}

void UKF::Prediction(double dt) {
    /**
        * Predicts sigma points, the state, and the state covariance matrix.
        * @param {double} dt the change in time (in seconds) between the last
        * measurement and this one.
    */
    MatrixXd Xsig_aug = this->GenerateAugmentedSigmaPoints();
    MatrixXd Xsig_pred = this->SigmaPointPrediction(Xsig_aug, dt);
    this->PredictMeanAndCovariance(Xsig_pred);

    // Assign the value to the class to reuse it later
    Xsig_pred_  = Xsig_pred;
}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
        Updates the state and the state covariance matrix using a radar measurement.
        @param {MeasurementPackage} meas_package

        Kalman Gain filter:
            K(k+1|k) = T(k+1|k).S(-1, k+1|k)
        State update:
            x(k+1|k+1) = x(k+1|k+1) + K(k+1|k).(z(k+1) - z(k+1|k))
        Covariance Matrix update:
            P(k+1|k+1) = P(k+1|k) - K(k+1|k).S(k+1|k).K(T,k+1|k)

        New for the Unscented kalman filter:
        CrossCorrelation between sigma in state space and measurement
        space:
            T(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
     */
    int n_z = 2;
    // Prediction of S(-1, k+1|k) and z(k+1|k)
    this->PredictLaserMeasurement();

     // Store the measurement into a new vector
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

    /*
       Create matrix for cross correlation Tc
       T(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
   */
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual: (Z(k+1|k,i) - z(k+1|k))
        VectorXd z_diff = Zsig_laser_.col(i) - z_pred_laser_;
        // state difference: (X(k+1|k,i) - x(k+1|k))
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //  w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_laser_.inverse();
    //residual
    VectorXd z_diff = z - z_pred_laser_;

    /*
        Log NIS values for this sensor
    */
    double nis = (z - z_pred_laser_).transpose()*S_laser_.inverse()*(z - z_pred_laser_);
    // Log true values vs predicted values
    Tools::logNisValue(nis, "laser_nis.log");
    Tools::logValue(z(0), z_pred_laser_(0), "laser_px.log");
    Tools::logValue(z(1), z_pred_laser_(1), "laser_py.log");

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S_laser_*K.transpose();

}

void UKF::PredictLaserMeasurement(){
    /*
        We are going to reuse our current predicted sigma point to predict
        the value of the next measurement.

        We start from sigma point with this state: [px, py, v, φ, φ.]
        and we generate new sigma point with this state: [ρx, py]
    */
    int n_z = 2; // New size of the state vector ([ρx, py])
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        Zsig(0,i) =  Xsig_pred_(0,i);  // px
        Zsig(1,i) =  Xsig_pred_(1,i); // py
    }

    /*
        Predict the state mean vector
        z(k+1|k) = Σ(no, i=0) w(i)Z(k+1|k,i)
    */
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    /*
        Predict the new covariance matrix
        S(k+1|k) = Σ(2.no, i=0) w(i).(Z(k+1|k,i) - z(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T + R
        (Note: The difference here is the + R which is the measurement noise)
    */
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    // S(k+1|k) = Σ(2.no, i=0) w(i).(Z(k+1|k,i) - z(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T + R
    // We add the "+R", the measurement noise
    S = S + R;

    S_laser_ = S;
    z_pred_laser_ = z_pred;
    Zsig_laser_ = Zsig;
}

void UKF::PredictRadarMeasurement(){
    /*
        We are going to reuse our current predicted sigma point to predict
        the value of the next measurement.

        We start from sigma point with this state: [px, py, v, φ, φ.]
        and we generate new sigma point with this state: [ρ, φ, ρ.]
    */

    int n_z = 3; // New size of the state vector ([ρ, φ, ρ.])
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i); // px
        double p_y = Xsig_pred_(1,i); // py
        double v  = Xsig_pred_(2,i); // v
        double yaw = Xsig_pred_(3,i); // φ

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y); // ρ
        Zsig(1,i) = atan2(p_y,p_x); // φ
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y); // ρ.
    }

    /*
        Predict the state mean vector
        z(k+1|k) = Σ(no, i=0) w(i)Z(k+1|k,i)
    */
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    /*
        Predict the new covariance matrix
        S(k+1|k) = Σ(2.no, i=0) w(i).(Z(k+1|k,i) - z(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T + R
        (Note: The difference here is the + R which is the measurement noise)
    */
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // (Z(k+1|k,i) - z(k+1|k))
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        // w(i) . (Z(k+1|k,i) - z(k+1|k)) . (Z(k+1|k,i) - z(k+1|k))T
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<  std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;

    // S(k+1|k) = Σ(2.no, i=0) w(i).(Z(k+1|k,i) - z(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T + R
    // We add the "+R", the measurement noise
    S = S + R;

    S_radar_ = S;
    z_pred_radar_ = z_pred;
    Zsig_radar_ = Zsig;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
        Updates the state and the state covariance matrix using a radar measurement.
        @param {MeasurementPackage} meas_package

        Kalman Gain filter:
            K(k+1|k) = T(k+1|k).S(-1, k+1|k)
        State update:
            x(k+1|k+1) = x(k+1|k+1) + K(k+1|k).(z(k+1) - z(k+1|k))
        Covariance Matrix update:
            P(k+1|k+1) = P(k+1|k) - K(k+1|k).S(k+1|k).K(T,k+1|k)

        New for the Unscented kalman filter:
        CrossCorrelation between sigma in state space and measurement
        space:
            T(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
     */
     int n_z = 3;

     // Prediction of S(-1, k+1|k) and z(k+1|k)
     this->PredictRadarMeasurement();

     // Store the measurement into a new vector
     VectorXd z = VectorXd(n_z);
     z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

     /*
        Create matrix for cross correlation Tc
        T(k+1|k) = Σ(2.no, i=0) w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
    */
     MatrixXd Tc = MatrixXd(n_x_, n_z);
     //calculate cross correlation matrix
     Tc.fill(0.0);
     for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual: (Z(k+1|k,i) - z(k+1|k))
        VectorXd z_diff = Zsig_radar_.col(i) - z_pred_radar_;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        // state difference: (X(k+1|k,i) - x(k+1|k))
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        //  w(i).(X(k+1|k,i) - x(k+1|k)).(Z(k+1|k,i) - z(k+1|k))T
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_radar_.inverse();
    //residual
    VectorXd z_diff = z - z_pred_radar_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    /*
        Log NIS values for this sensor
    */
    double nis = (z - z_pred_radar_).transpose()*S_radar_.inverse()*(z - z_pred_radar_);
    // Log true values vs predicted values
    Tools::logNisValue(nis, "radar_nis.log");
    Tools::logValue(z(0), z_pred_radar_(0), "radar_p.log");
    Tools::logValue(z(1), z_pred_radar_(1), "radar_rho.log");
    Tools::logValue(z(2), z_pred_radar_(2), "radar_p_dot.log");

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S_radar_*K.transpose();
}
