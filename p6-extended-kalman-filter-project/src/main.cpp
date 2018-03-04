#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using json = nlohmann::json;

std::string hasData(std::string s) {
    /*
        Checks if the SocketIO event has JSON data.
        If there is data the JSON object in string format will be returned
        else the empty string "" will be returned.
    */
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos)
        return "";
    else if (b1 != std::string::npos && b2 != std::string::npos)
        return s.substr(b1, b2 - b1 + 1);
    return "";
}

int main() {
    uWS::Hub h;
    FusionEKF fusionEKF; // Kalman Filter instance
    Tools tools; // Usesefull class

    // used to compute the RMSE later
    // Vector used to list all predicted value at each time step
    vector<VectorXd> estimations;
    // Vector used to list all true value at each time step
    vector<VectorXd> ground_truth;

    h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        /*
            *length: Size of the message (data)
            *data[0] and data[1]
                "42" at the start of the message means there's a websocket
                message event.
                The 4 signifies a websocket message
                The 2 signifies a websocket event
        */
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            //  Keep only the json part of the string
            auto s = hasData(std::string(data));
            if (s != "") {
                // Json to String
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                  // j[1] is the data JSON object
                  string sensor_measurment = j[1]["sensor_measurement"];

                  MeasurementPackage meas_package;
                  istringstream iss(sensor_measurment);
                  long long timestamp;

                  // reads first element from the current line
                  string sensor_type;
                  iss >> sensor_type;

                  // Extract all informations from either the Lidar measurement
                  // Or the Radar measurement
                  if (sensor_type.compare("L") == 0) {
                        float px;
                        float py;
                	  	meas_package.sensor_type_ = MeasurementPackage::LASER;
                  		meas_package.raw_measurements_ = VectorXd(2);
                  		iss >> px;
                  		iss >> py;
                  		meas_package.raw_measurements_ << px, py;
                  		iss >> timestamp;
                  		meas_package.timestamp_ = timestamp;
                  } else if (sensor_type.compare("R") == 0) {
                        float ro;
                        float theta;
                        float ro_dot;
                        meas_package.sensor_type_ = MeasurementPackage::RADAR;
                  		meas_package.raw_measurements_ = VectorXd(3);
                  		iss >> ro;
                  		iss >> theta;
                  		iss >> ro_dot;
                  		meas_package.raw_measurements_ << ro,theta, ro_dot;
                  		iss >> timestamp;
                  		meas_package.timestamp_ = timestamp;
                  }
                  // Ground truth values
                  float x_gt;
                  float y_gt;
                  float vx_gt;
                  float vy_gt;
                  iss >> x_gt;
                  iss >> y_gt;
                  iss >> vx_gt;
                  iss >> vy_gt;
                  VectorXd gt_values(4);
                  gt_values(0) = x_gt;
                  gt_values(1) = y_gt;
                  gt_values(2) = vx_gt;
                  gt_values(3) = vy_gt;
                  ground_truth.push_back(gt_values);

                  //Call ProcessMeasurment(meas_package) for Kalman filter
                  fusionEKF.processMeasurement(meas_package);
                  //Push the current estimated x,y positon from the Kalman filter's state vector
                  VectorXd estimate(4);

                  // Predicted value from the Fusion Kalman filter algorithm
                  double p_x = fusionEKF.ekf.x(0); // px (Position in x)
                  double p_y = fusionEKF.ekf.x(1); // py (Position in y)
                  double v1  = fusionEKF.ekf.x(2); // vx (Velocity in x)
                  double v2 = fusionEKF.ekf.x(3); // vy (Velocity in y)

                  // Add theses values to the estimate vector
                  estimate(0) = p_x;
                  estimate(1) = p_y;
                  estimate(2) = v1;
                  estimate(3) = v2;
                  // Push this prediction into the estimations vector to compute
                  // RSME.
                  estimations.push_back(estimate);

                  // Compute the current RSME
                  VectorXd RMSE = Tools::calculateRMSE(estimations, ground_truth);

                  // Create a new JSON to send to the server
                  json msgJson;
                  msgJson["estimate_x"] = p_x;
                  msgJson["estimate_y"] = p_y;
                  msgJson["rmse_x"] =  RMSE(0);
                  msgJson["rmse_y"] =  RMSE(1);
                  msgJson["rmse_vx"] = RMSE(2);
                  msgJson["rmse_vy"] = RMSE(3);
                  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                  // std::cout << msg << std::endl;
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else {
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }

      });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
