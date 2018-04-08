
## Particle Filtere 
In this project, I implemented a 2-dimensional particle filter in C++. The particle filter uses a map of the environment and some initial localization information (analogous to what a GPS would provide). At each time step, the filter also gets observation form the Lida sensor and control data (velocity and yaw rate).


This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

---

# Particle filter implementation

The implementation of the Particle filter is located into src/particle_filter.cpp. The is divive int the N following part ::

### Initialization of particiles [ParticleFilter::init]:
    Initialize all particles to their first position based on the 
    x, y, theta estimations from the GPS).
    A random gaussian is add to each particle.

### Prediction [ParticleFilter::prediction]
    The prediction step used the last estimate velocity and yaw_rate measure form the vehicle to
    estimate the new position of the vehicle at time step t+1.

### Update weights [ParticleFilter::updateWeights]
    Update the weights of each particle using a multivariate Gaussian distribution. The observations are given in the     
    VEHICLE'S coordinate system. Then, I translate each observations to the MAP's coordinate system. I compute each weight 
    according to the nearest landmark from each observation.

### Resample [void ParticleFilter::resample]
    Resample particles according to their weights.
    The resampling is done using the wheel method.
    

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.



