
#ifndef STRUCT_H
#define STRUCT_H

#include "json.hpp"

using namespace std;

enum state_e { LEFT, RIGHT, STRAIGHT };

struct car_s {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

struct generatedpath_s{
    vector<double> x;
    vector<double> y;
};

struct path_s {
    nlohmann::basic_json<>::value_type x;
    nlohmann::basic_json<>::value_type y;
    double end_s;
    double end_d;
};

#endif /* STRUCT_H */
