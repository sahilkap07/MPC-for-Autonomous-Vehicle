#include <iostream>
#include "trajectory.h"

Trajectory::Trajectory(Maneuver track, double xvel, double time, double ts) : Track(track), XVel(xvel), Time(time), Ts(ts) {
	XPts.resize(int(Time / Ts) + 1);
	YPts.resize(int(Time / Ts) + 1);
	Psi.resize(int(Time / Ts) + 1);
}

std::tuple<Eigen::ArrayXd, Eigen::ArrayXd, Eigen::ArrayXd> Trajectory::GetTrajectoryPts() {

	XPts.setLinSpaced(int(Time / Ts) + 1, 0, XVel * Time);

	switch (Track) {

	case Maneuver::Straight: {
		YPts.setOnes();
		YPts *= -9;
		break;
	}

	case Maneuver::LaneChange: {
		YPts.setLinSpaced((Time / Ts) + 1, 0, Time);
		YPts = 9 * (YPts - Time / 2).tanh();
		break;
	}

	case Maneuver::ObstacleAvoidance: {
		Eigen::ArrayXd y1 = -(0.002545 * (XPts - 100).pow(2)) + 14;
		Eigen::ArrayXd y2 = 2 * 4 * (2 * M_PI * 0.01 * XPts).sin();
		YPts = (y1 + y2) / 2;
		break;
	}

	default: std::cout << "Not a valid trajectory." << std::endl;
	}

	Eigen::ArrayXd dX = XPts.tail(XPts.size() - 1) - XPts.head(XPts.size() - 1);
	Eigen::ArrayXd dY = YPts.tail(YPts.size() - 1) - YPts.head(YPts.size() - 1);

	Psi(0) = atan(dY(0) / dX(0));
	Psi.tail(Psi.size() - 1) = (dY.cwiseQuotient(dX)).atan();
	Eigen::ArrayXd dPsi = Psi.tail(Psi.size() - 1) - Psi.head(Psi.size() - 1);

	for (int i = 1; i < Psi.size(); i++) {
		if (dPsi(i - 1) < -M_PI)
			Psi(i) = Psi(i - 1) + (dPsi(i - 1) + 2 * M_PI);
		else if (dPsi(i - 1) > M_PI)
			Psi(i) = Psi(i - 1) + (dPsi(i - 1) - 2 * M_PI);
		else
			Psi(i) = Psi(i - 1) + dPsi(i - 1);
	}
	return { XPts, YPts, Psi };
}