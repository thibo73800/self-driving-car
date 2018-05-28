#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  double throttle;
  double steering_angle;
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;


  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  // Convert the waypoints point to the vehicle coordinate system
  tuple<Eigen::VectorXd, Eigen::VectorXd> WaypointsToVehiclesPoints(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi);
  // Kinematic model: Prediction of the next state
  Eigen::VectorXd kinematicModel(Eigen::VectorXd vector, double dt, double throttle, double steering_angle);
};

#endif /* MPC_H */
