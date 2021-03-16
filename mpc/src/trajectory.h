#pragma once
#include <math.h>
#include <vector>
#include <tuple>
#include <algorithm>
#include <functional>
#include <Eigen/Dense>

enum class Maneuver { Straight = 1, LaneChange, ObstacleAvoidance };

class Trajectory
{
private:
	Maneuver Track;
	double XVel, Time, Ts;
	Eigen::ArrayXd XPts, YPts, Psi;

public:
	Trajectory(Maneuver track, double xvel, double time, double ts);
	std::tuple<Eigen::ArrayXd, Eigen::ArrayXd, Eigen::ArrayXd> GetTrajectoryPts();
};