#include <iostream>
#include <math.h>
#include <fstream>

#include "utils.h"
#include "path_planner.h"
#include "spline.h"
#include "structs.h"

using lidar = nlohmann::basic_json<std::map, std::vector, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, bool, long, unsigned long, double, std::allocator, nlohmann::adl_serializer>;


PathPlanner::PathPlanner(){

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
      istringstream iss(line);
      double x, y;
      float s, d_x, d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      this->map_waypoints_x.push_back(x);
      this->map_waypoints_y.push_back(y);
      this->map_waypoints_s.push_back(s);
      this->map_waypoints_dx.push_back(d_x);
      this->map_waypoints_dy.push_back(d_y);
    }

    this->velocity = 0;
    this->max_velocity = 49.5;
    this->safe_dist = 20;
    this->lane = 1;
    this->target_lane = 1;
};

PathPlanner::~PathPlanner(){};

/**
	Get the speed of the lane
	-1 if the lane is empty
	>= 0 if cars are on the lane
**/
double PathPlanner::getLaneSpeed(double lane, const car_s &car, lidar &sensor_fusion){
    // find ref_v to use

	double lane_speed = this->max_velocity;

    for (int i=0; i < sensor_fusion.size(); i++){
        // car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2+4*lane+2) && d > (2+4*lane-2)){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
			double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
			double vehicle_s = sensor_fusion[i][5];

			// Speed of the vehicle
            double check_speed = sqrt((vx*vx)+(vy*vy))*2.24;

			// Get the futur s and d position in the Frenet coordinates
			double nx = x+(vx*0.02);
			double ny = y+(vy*0.02);
            vector<double> nsd = getFrenet(nx, ny, 0, this->map_waypoints_x, this->map_waypoints_y);
            double predicted_vehicle_s = nsd[0];

			if ( ((vehicle_s > car.s) && (vehicle_s-car.s < this->safe_dist)) /*|| ((predicted_vehicle_s >= car.s)   && (predicted_vehicle_s-car.s < this->safe_dist)) */){
                lane_speed = check_speed - 0.1*(this->safe_dist-(vehicle_s-car.s));
                //std::cout << "vehicle_s: " << vehicle_s << "\t predicted_vehicle_s: " << predicted_vehicle_s << "\tvelocity: " << check_speed << "\ttarget speed: " << lane_speed << '\n';
            }
        }
    }
	return lane_speed;
}


/*
    Check if the line is free
*/
bool PathPlanner::isLineFree(double lane, const car_s &car, lidar &sensor_fusion){

	double lane_speed = -1;

    for (int i=0; i < sensor_fusion.size(); i++){
        // car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
			double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
			double vehicle_s = sensor_fusion[i][5];

            // Size of the car
            vehicle_s = vehicle_s;

			// Speed of the vehicle
            double check_speed = sqrt((vx*vx)+(vy*vy))*2.24;

			// Get the futur s and d position in the Frenet coordinates
			double nx = x+(vx*0.02);
			double ny = y+(vy*0.02);
            vector<double> nsd = getFrenet(nx, ny, 0, this->map_waypoints_x, this->map_waypoints_y);
            double predicted_vehicle_s = nsd[0];

			if (((vehicle_s >= car.s - 15) && (vehicle_s < car.s + this->safe_dist)) || ((predicted_vehicle_s >= car.s - 15)  && (predicted_vehicle_s < car.s + this->safe_dist)) ){
                return false;
            }
        }
    }
	return true;
}

generatedpath_s PathPlanner::generateTrajectory(const path_s &previous_path, const car_s &car, const state_e &state, lidar &sensor_fusion, bool full, double lane){
		if (lane < 0){
			lane = this->lane;
		}
		// fixe ptsX, fixe ptsY
    	vector<double> ptsx;
    	vector<double> ptsy;

		double refx = car.x;
    	double refy = car.y;
    	double ref_yaw = deg2rad(car.yaw);
    	int prev_size = previous_path.x.size();
		generatedpath_s n_path;

	    // if previous size is almost empty
	    // use the car as a starting point
	    if (full || prev_size < 2){
	        // Go backward to see where the car was before
	        double prev_car_x = car.x - cos(car.yaw);
	        double prev_car_y = car.y - sin(car.yaw);
	        std::cout << "previous car x: " << prev_car_x << '\n';
	        std::cout << "previous car y: " << prev_car_y << '\n';
	        // Current position, x and y
	        ptsx.push_back(car.x);
	        ptsy.push_back(car.y);
	    }
	    // Use the previous cars points as a starting reference
	    else {
	        // Ref is the position before
	        refx = previous_path.x[prev_size - 1];
	        refy = previous_path.y[prev_size - 1];
	        double refx_prev = previous_path.x[prev_size - 2];
	        double refy_prev = previous_path.y[prev_size - 2];
	        // Decude the yan from the two previous points
	        ref_yaw = atan2(refy-refy_prev, refx-refx_prev);
	        ptsx.push_back(refx_prev);
	        ptsx.push_back(refx);
	        ptsy.push_back(refy_prev);
	        ptsy.push_back(refy);
	    }

	    // In frenet add 30 points ahead of the starting reference
	    vector<double> next_wp0 = getXY(car.s+30, (2+4*lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
	    vector<double> next_wp1 = getXY(car.s+60, (2+4*lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
	    vector<double> next_wp2 = getXY(car.s+90, (2+4*lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);

	    ptsx.push_back(next_wp0[0]);
	    ptsx.push_back(next_wp1[0]);
	    ptsx.push_back(next_wp2[0]);

	    ptsy.push_back(next_wp0[1]);
	    ptsy.push_back(next_wp1[1]);
	    ptsy.push_back(next_wp2[1]);

	    for(int i = 0; i < ptsx.size(); i++) {
	        double shift_x = ptsx[i]-refx;
	        double shift_y = ptsy[i]-refy;
	        // Shift the new positions wich respect to the car coordinates
	        ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0- ref_yaw));
	        ptsy[i] = (shift_x*sin(0 - ref_yaw)+shift_y*cos(0-ref_yaw));
	    }
	    tk::spline s;
	    s.set_points(ptsx, ptsy);

		if (!full){
			// Start with all the previous points path
		    for (int i=0; i< previous_path.x.size(); i++){
		        n_path.x.push_back(previous_path.x[i]);
		        n_path.y.push_back(previous_path.y[i]);
		    }
		}

	    double target_x = 30.00;
	    double target_y = s(target_x);
	    double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
	    double x_add_on = 0;
		double N=(target_dist/(0.02*this->velocity/2.24));

		if (full){
			prev_size = 0;
		}

	    for (int i=0; i<=40-prev_size; i++){
	        double x_point = x_add_on+(target_x)/N;
	        double y_point = s(x_point);
	        x_add_on = x_point;
	        double xpt = x_point;
	        double ypt = y_point;
	        x_point = (xpt*cos(ref_yaw)-ypt*sin(ref_yaw));
	        y_point = (xpt*sin(ref_yaw)+ypt*cos(ref_yaw));
	        x_point += refx;
	        y_point += refy;
	        n_path.x.push_back(x_point);
	        n_path.y.push_back(y_point);
	    }

	    return n_path;
}

/*
    Check the three lines to detect which one is free and the fastest.
*/
generatedpath_s PathPlanner::checkPath(const path_s &previous_path, const car_s &car, const state_e &state, lidar &sensor_fusion){
    std::cout << "--------" << '\n';

	double lane = this->lane;

	double speeds[3] = {
            getLaneSpeed(0, car, sensor_fusion),
            getLaneSpeed(1, car, sensor_fusion),
            getLaneSpeed(2, car, sensor_fusion)
    };

    double lane_speed = getLaneSpeed(this->target_lane, car, sensor_fusion);

    int quickest_line = this->lane;
    for (int i = 0; i < 3; i++) {
        if (this->velocity > 20. && i != target_lane && (speeds[i] > lane_speed || (speeds[i] == lane_speed && i > target_lane))) {
            int new_line = this->target_lane + std::fmax(-1.0, std::fmin(i - this->lane, 1.0));
            if (this->isLineFree(new_line, car, sensor_fusion)){
                std::cout << "=>>>>Change target line:" << " new_line: " << new_line << " speeds[i]: " << speeds[i] << " lane_speed: " << lane_speed << " target_lane: " << this->target_lane << "\t isLineFree: " <<  this->isLineFree(new_line, car, sensor_fusion) <<'\n';
                this->target_lane = new_line;
            }
        }
    }

    // Set the target lane
    std::cout << "Target lane: " <<  this->target_lane << '\n';
    double line_change = std::fmax(-1.0, std::fmin(this->target_lane - this->lane, 1.0));
    this->lane =  this->lane + (0.05*line_change);

    // Speed of the target lane
    lane_speed = getLaneSpeed(this->lane, car, sensor_fusion);

    if (this->velocity >= lane_speed && lane_speed != -1){
        std::cout << "Keep the car speed: " << lane_speed << '\n';
        this->velocity = std::fmax(this->velocity - 1, lane_speed);
    }
    else {
        std::cout << "Go to max velocity: " << this->max_velocity << '\n';
        this->velocity = std::fmin(this->velocity + 0.15, lane_speed);
    }

    std::cout << "Lane:" << this->lane << '\n';
    std::cout << "Current speed: " << this->velocity << '\n';

    // Generated the new path according to the target lane and target speed
	generatedpath_s new_path = generateTrajectory(previous_path, car, state, sensor_fusion, false, this->lane);

    return new_path;
}
