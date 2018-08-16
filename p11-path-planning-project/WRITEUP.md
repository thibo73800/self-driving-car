# Write Up

## The car drives according to the speed limit.

The speed of the car can be controlled through the space between each waypoint of the path. Then we can make a conversion and a
division in order to set the gap between each waypoint.

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L201

```[cpp]
double N=(target_dist/(0.02*this->velocity/2.24));
```

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L208

```[cpp]
double x_point = x_add_on+(target_x)/N;
```

## Max Acceleration and Jerk are not Exceeded.

To not exceed the maximum speed and the maximum jerk the path of the car is smooth such that there are not brutal changes from
one generation to another. Moreover, I also smooth the desired speed of the vehicle to make sure I do not change the speed too
quickly between one line and another.

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L259

```[cpp]
if (this->velocity >= lane_speed && lane_speed != -1){
    this->velocity = std::fmax(this->velocity - 1, lane_speed);
}
else {
    this->velocity = std::fmin(this->velocity + 0.15, lane_speed);
}
```

## Car does not have collisions.

To avoid collisions I first check the speed of other vehicles on the current lane and make an estimation of the vehicle speed.
Once the estimation is done, I set the desired speed of the car according to the car on the lane and the current distance. Thus
the vehicle goes slower if the gap between the two vehicles is too low.

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L70


```[cpp]
double check_speed = sqrt((vx*vx)+(vy*vy))*2.24;
```

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L79


```[cpp]
lane_speed = check_speed - 0.1*(this->safe_dist-(vehicle_s-car.s));
```

Finally, I also have a method to check whether a line is free or not. This method checks the detected vehicles above and bellow
the autonomous vehicles to make sure it is possible to change the lane.

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L91

```[cpp]
bool PathPlanner::isLineFree(double lane, const car_s &car, lidar &sensor_fusion)
```

## The car stays in its lane, except for the time between changing lanes.

To stay in its lane, the algorithm uses the spline library to generate the first equation to follow along the Frenet
coordinates of the desired lane (the s coordinate).

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L187

```[cpp]
tk::spline s;
s.set_points(ptsx, ptsy);
```

The ptsx and ptsy points above have been generate accordint to the s frenete coordinate.

## The car is able to change lanes

Because the algorithm smooth the number of points between two waypoints path, changing line is automatically smooth.
The algorithm check which lane is the fastest, check if it is possible to change the lane by calling the isLineFree method, If it is, the target lane is changed and new waypoints are generated toward the new lane.

https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L241

```[cpp]
for (int i = 0; i < 3; i++) {
    if (this->velocity > 20. && i != target_lane && (speeds[i] > lane_speed || (speeds[i] == lane_speed && i > target_lane))) {
        int new_line = this->target_lane + std::fmax(-1.0, std::fmin(i - this->lane, 1.0));
        if (this->isLineFree(new_line, car, sensor_fusion)){
            this->target_lane = new_line;
        }
    }
}
```

## Generate Paths

To generate the path I made a method called generateTrajectory https://github.com/thibo73800/self-driving-car/blob/master/p11-path-planning-project/src/path_planner.cpp#L125
The method set the waypoints according to the target lane, target speed, and the past waypoints. The code start by drawing a set of points (three) starting at the current car S coordinate and going to the target S. Then I used the spline library
to generate the exact line equation to follow. Once I have the equation of my line I can add the desired waypoints along
the road with a gap proportional to the target speed.



