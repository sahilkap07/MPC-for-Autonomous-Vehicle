# MPC-for-Autonomous-Vehicle
MPC on a bicycle model in C++

## Control Methodology : 

Model Predictive Control = Receding-Horizon Optimal Control + Constrained Optimal Control 

Lateral Control of the vehicle is achieved at constant forward velocity using a Model Predictive Controller for three scenarios:
1. Straight Line Control
2. Lane Change Maneuver
3. Obstacle Avoidance

### States of the system
* Lateral Velocity 
* Yaw
* Yaw Rate
* Y Position 
* Steering Angle (At previous time step - After Augmentation of SS Matrices)

### Input
* Steering Angle

## Code Implementation

C++ is used for the processing the data and acquiring new states using the MPC cost function. Eigen library is used for matrix calcultions. The project does not use any 
optimization library and the input steering commands are calculated by taking the derivative of the cost function and equating to zero. New states are then calculated using 
discrete time state-space equations.
Main file can be used to change the trajectory type to : Straight, LaneChange or ObstacleAvoidance. In each of the three cases, the data in the main file can be tweaked to 
observe the trajectory in different scenarios, such as vehicle speed, length of horizon period, time-step for calculation etc.

Plotting is done entirely in MATLAB. The data is passed from C++ to MATLAB by calling MATLAB Engine in C++ and copying the data in MATLAB environment. Animate.m is the 
matlab file which is called directly from C++ to run the Matlab Command Prompt and observe the simulation.

C++ 17 is required for the code to run as it uses the structured bindings which is not available is earlier versions of C++.
Eigen Library can be downloaded and included from [link](https://eigen.tuxfamily.org/index.php?title=Main_Page).

https://user-images.githubusercontent.com/59492146/113215452-dee85200-9248-11eb-8158-6ddd037e9c87.mp4




