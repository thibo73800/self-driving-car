#include "tools.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

// Constructor && Desconstructor
Tools::Tools() {}
Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
      Method to calculate the RMSE
    */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    /*
      Check the validity of the following inputs:
             - The estimation vector size should not be zero
             - The estimation vector size should equal ground truth vector size
    */
    if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    	cout << "Invalid estimation or ground_truth data" << endl;
    	return rmse;
    }
    // Accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
    	VectorXd residual = estimations[i] - ground_truth[i];
    	//coefficient-wise multiplication
    	residual = residual.array()*residual.array();
    	rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();
    //return the result
    return rmse;
}

VectorXd Tools::CartToPolar(const VectorXd &state){
    /*
        Method used to translate cartesian coordinate (px, py, vx, vy)
        to polar coordinate (p, tho, p_dot)
    */
    VectorXd polar = VectorXd(3);

    float px = state[0];
    float py = state[1];
    float vx = state[2];
    float vy = state[3];

    float p = sqrt(pow(px, 2) + pow(py, 2));
    float phi = atan2(py, px);
    float p_dot = ((px*vx)+(py*vy)) / p;

    polar << p, phi, p_dot;
    return polar;
}

VectorXd Tools::PolarToCart(const VectorXd &state) {
    /*
        Method used to translate polar coordinate (p, tho, p_dot) to
        cartesian coordinate (px, py, vx, vy);
    */
    VectorXd cart = VectorXd(5);

    const float p = state[0];
    const float phi = state[1];
    const float p_dot = state[2];

    // Trigonometri to decude the position of px and py
    const float px = p * cos(phi);
    const float py = p * sin(phi);

    cart << px, py, 0.0, 0.0, 0.0;
    return cart;
}

void Tools::emptyLogFiles(){
    ofstream myfile;

    myfile.open("radar_nis.log"); myfile << ""; myfile.close();
    myfile.open("radar_p.log"); myfile << ""; myfile.close();
    myfile.open("radar_rho.log"); myfile << ""; myfile.close();
    myfile.open("radar_p_dot.log"); myfile << ""; myfile.close();

    myfile.open("laser_nis.log"); myfile << ""; myfile.close();
    myfile.open("laser_px.log"); myfile << ""; myfile.close();
    myfile.open("laser_py.log"); myfile << ""; myfile.close();
}

void Tools::logNisValue(double nis, const std::string &file_name){
    std::ofstream out;
    out.open(file_name, std::ios::app);

    std::ostringstream strs;
    strs << nis;
    std::string str = strs.str();

    out << str;
    out << ",";
    out.close();
}

void Tools::logValue(double val1, double val2, const std::string &file_name){
    std::ofstream out;
    out.open(file_name, std::ios::app);

    std::ostringstream strs1;
    strs1 << val1;
    std::string str1 = strs1.str();

    std::ostringstream strs2;
    strs2 << val2;
    std::string str2 = strs2.str();

    out << str1;
    out << ";";
    out << str2;
    out << ",";
    out.close();
}
