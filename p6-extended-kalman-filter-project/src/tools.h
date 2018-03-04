#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
    public:
        // Constructor && Desconstructor
        Tools();
        virtual ~Tools();

        // Method used to compute the RSME for each state value
        static VectorXd calculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
        // Compute the Jacobian matrix (Hj)
        static MatrixXd calculateJacobian(const VectorXd& x_state);
        // Translate cartesian coordinate to polar coordinate
        static VectorXd cartToPolar(const VectorXd &state);
        // Translate polar coordinate to cartesian coordinate
        static VectorXd polarToCart(const VectorXd &state);
};

#endif /* _TOOLS_H_ */
