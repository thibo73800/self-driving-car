#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
    public:
        // Constructor && PolarToCart
        Tools();
        virtual ~Tools();

        // Method used to compute the RSME for each state value
        static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
        // Translate cartesian coordinate to polar coordinate
        static VectorXd CartToPolar(const VectorXd &state);
        // Translate polar coordinate to cartesian coordinate
        static VectorXd PolarToCart(const VectorXd &state);
};

#endif /* _TOOLS_H_ */
