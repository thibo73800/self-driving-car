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

        // Usefull method to log the result of this algorithm
        static void logNisValue(double nis, const std::string &file_name);
        static void logValue(double val1, double val2, const std::string &file_name);
        static void emptyLogFiles();
};

#endif /* _TOOLS_H_ */
