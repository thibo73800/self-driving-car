#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
    public:
        // Constructor && Desconstructor
        FusionEKF();
        virtual ~FusionEKF();
        // Process each measurement
        void processMeasurement(const MeasurementPackage &measurement_pack);

        // Use to store variables (Matrix and vector) and method for the Kalman Filter
        KalmanFilter ekf;

    private:
        // check whether the tracking toolbox was initialized or not (first measurement)
        bool is_initialized;
        // previous timestamp
        long long previous_timestamp;

        // Update the time from the last measurement
        float updateTime(const MeasurementPackage &measurement_pack);
        // Update the process covariance matrix Q
        float updateProcessCovariance(float dt);
        // Estimation part of the Kalman filter (Prediction + Correction)
        void  estimation(const MeasurementPackage &measurement_pack);
};

#endif /* FusionEKF_H_ */
