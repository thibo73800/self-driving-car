#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 15;
double dt = 0.08;

double x_start = 0;
double y_start = N;
double psi_start = N*2;
double v_start = N*3;
double cte_start = N*4;
double epsi_start = N*5;
double delta_start = N*6;
double a_start = (N*6)+(N-1);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double speed;
  FG_eval(Eigen::VectorXd coeffs, double speed) {
      this->coeffs = coeffs;
      this->speed = speed;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    AD<double> lf = Lf;
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 20000*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 200000*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - 100, 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 700000 * (this->speed+1) * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    }

    // Add constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t = 1; t < N; t++) {
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> x1 = vars[x_start + t];

        AD<double> y0 = vars[y_start + t - 1];
        AD<double> y1 = vars[y_start + t];

        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> psi1 = vars[psi_start + t];

        AD<double> v0 = vars[v_start + t - 1];
        AD<double> v1 = vars[v_start + t];

        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];

        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> cte1 = vars[cte_start + t];

        AD<double> epsi0 = vars[epsi_start + t - 1];
        AD<double> epsi1 = vars[epsi_start + t];

        AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*CppAD::pow(x0, 2) + coeffs[3]*CppAD::pow(x0, 3);
        AD<double> psides0 = CppAD::atan(coeffs[1] + 2*x0*coeffs[2] + 3*coeffs[3]*CppAD::pow(x0, 2));

        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 - (v0/lf) * delta0 * dt);
        fg[1 + v_start + t] = v1 - (v0 +  a0*dt);

        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = N*6+(N-1)*2;
  size_t n_constraints = N*6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for ( int i = 0; i < delta_start; i++ ) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Lower and upper limits for variables.
  for (int i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
   }
  for (int i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -1;
      vars_upperbound[i] = 1;
   }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;

  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;

  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;

  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;

  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;

  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, v);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;

  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;

  for (int i=0; i < N; i++){
     mpc_x_vals.push_back(solution.x[x_start+i]);
     mpc_y_vals.push_back(solution.x[y_start+i]);
  }

  this->mpc_x_vals = mpc_x_vals;
  this->mpc_y_vals = mpc_y_vals;

  return {solution.x[delta_start], solution.x[a_start]};
}

tuple<Eigen::VectorXd, Eigen::VectorXd>MPC::WaypointsToVehiclesPoints(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi){
    /*
        Convert the waypoints (ptsx) and ptsy
        Into the vehicle coordinate system
    */
    Eigen::VectorXd waypoints_x(ptsx.size());
    Eigen::VectorXd waypoints_y(ptsy.size());
    double cos_psi = cos(0.0-psi);
    double sin_psi = sin(0.0-psi);
    for (size_t i=0; i < ptsx.size(); i++){
        // x = dx.cos(φ) - dy.sin(φ)
        waypoints_x[i] = (ptsx[i] - px) * cos_psi - (ptsy[i] - py) * sin_psi;
        // y = dy.cos(φ) + dx.sin(φ)
        waypoints_y[i] = (ptsx[i] - px) * sin_psi + (ptsy[i] - py) * cos_psi;
    }
    return make_tuple(waypoints_x, waypoints_y);
}

Eigen::VectorXd MPC::kinematicModel(Eigen::VectorXd state, double dt, double throttle, double steering_angle){
    double x = state(0);
    double y = state(1);
    double psi = state(2);
    double v = state(3);
    double cte = state(4);
    double epsi = state(5);

    double nx = x + v*cos(psi)*dt;
    double ny = y + v*sin(psi)*dt;
    double npsi = psi + (v/Lf)*steering_angle*dt;
    double nv = v + throttle*dt;
    double ncte = cte + v*sin(epsi)*dt;
    double nepsi = epsi + (v/Lf)*steering_angle*dt;

    Eigen::VectorXd n_state(6);
    n_state << nx, ny, npsi, nv, ncte, nepsi;
    return n_state;
}
