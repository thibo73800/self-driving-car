#include "structs.h"

#ifndef PATH_PLANER_H
#define PATH_PLANER_H

using lidar = nlohmann::basic_json<std::map, std::vector, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, bool, long, unsigned long, double, std::allocator, nlohmann::adl_serializer>;

class PathPlanner {

public:
    PathPlanner();
    ~PathPlanner();
    generatedpath_s checkPath(const path_s &previous_path, const car_s &car, const state_e &state, lidar &sensor_fusion);
    double getLaneSpeed(double lane, const car_s &car, lidar &sensor_fusion);
    generatedpath_s generateTrajectory(const path_s &previous_path, const car_s &car, const state_e &state, lidar &sensor_fusion, bool full, double lane = -1);
    bool isLineFree(double lane, const car_s &car, lidar &sensor_fusion);

private:
    double velocity = 0;
    double max_velocity = 49;
    double safe_dist = 20;
    double lane = 1;
    int target_lane = 1;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


};


#endif /* PATH_PLANER_H */
