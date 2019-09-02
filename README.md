# Particle Filter
Self-Driving Car Engineer Nanodegree Program

[//]: # "Image References"

[image1]: ./images/summary.png

## Overview
In this localization project a **2 dimensional particle filter** is implemented in C++. Particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation from sensors and control data, both of them contain noise, modelled by a gaussian distribution. Detailed explanation of the code is provided below.

**INPUT**: values provided by the simulator to the c++ program

- sense noisy position data from the simulator (Analogy to noisy GPS data)
  `["sense_x"]`, `["sense_y"]`, `["sense_theta"]`

- previous velocity and yaw rate to predict the particle's transitioned state
  `["previous_velocity"]`, `["previous_yawrate"]`

- receive noisy observation data of the landmarks from the simulator
  `["sense_observations_x"]`, `["sense_observations_y"]`


**OUTPUT** values provided by the c++ program to the simulator

- best particle values used for calculating the error evaluation
  `["best_particle_x"]`, `["best_particle_y"]`, `["best_particle_theta"]`

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

## Build Instructions
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

#### Dependencies

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

## Code explanation
Code is in /src/particle_filter.cpp. Here is a chart summerizing the process loop:
![alt text][image1]

1. **Initialization**: `(/src/particle_filter.cpp line 28 - 54)`
    create **N** particles based on GPS position and its standard deviation (each particle represents a guess of the true vehicle position). The subsequent steps in the process will refine these guesses to localize the vehicle.
```cpp
/**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   */
```
2. **Movement Prediction**: `(/src/particle_filter.cpp line 56 - 87)`
    For all particles, vehicle control input with noise (yaw rate & velocity) is added.
```cpp
/*
    * if the turning rate is large:
    *       Apply equations of motion model (turning)
    * else if the turning rate is small:
    *       Apply equations of motion model (linear) (this avoids division by zero error)
    * 
    * add noise to prediction
*/
```
3. **Observation**: `(/src/particle_filter.cpp line 116 - 204)`
    - using sensor to obtain observations `obs` of the landmark in *vehicle coordinate*, sensor itself has measurement uncertainty, modelled using standard deviation as `std_sensor`
    - obtain positions of the landmark in *global cartesian coordinate* as `map feature`
    - for each particle (possible position of the vehicle):
        - transform `obs` into *global cartesian coordinate* as `obs_map_coord`
        - using **nearest neighbour** to associate `obs_map_coord ` and `map feature`
        - compute gaussian probability `prob` using `obs_map_coord`, `map_feature` and `std_sensor`
        - compute the `weight` of this particle by multiplying together all `prob`
4. **Normalize** all particle weights so that particle weights add up to 1
5. **Resample**: `(/src/particle_filter.cpp line 205 - 232)`
    resample **N** times (N is the number of particles) drawing a particle **i** (i is the particle index) proportional to its weight

