#include "vehicle_model.h"

VehicleModel::VehicleModel() : Mass(1500), Iz(3000), Caf(19000), Car(33000), Lf(2), Lr(3), XVel(20) {}

VehicleModel::VehicleModel(int mass, int iz, double caf, double car, double lf, double lr, double xVel) : Mass(mass), Iz(iz), Caf(caf), Car(car), Lf(lf), Lr(lr), XVel(xVel) {}

// to set the vehicle state space model i.e. matrices A, B, C, D
StateMatrices VehicleModel::GetStateSpace(double ts) {
	SS.A.setZero(4, 4);
	SS.A(0, 0) = -(2 * Caf + 2 * Car) / (Mass * XVel);
	SS.A(0, 2) = -XVel - (2 * Caf * Lf - 2 * Car * Lr) / (Mass * XVel);
	SS.A(1, 2) = 1;
	SS.A(2, 0) = -(2 * Lf * Caf - 2 * Lr * Car) / (Iz * XVel);
	SS.A(2, 2) = -(2 * pow(Lf, 2) * Caf + 2 * pow(Lr, 2) * Car) / (Iz * XVel);
	SS.A(3, 0) = 1;
	SS.A(3, 1) = XVel;
	SS.A = Eigen::MatrixXd::Identity(SS.A.rows(), SS.A.cols()) + ts * SS.A;

	SS.B << 2 * Caf / Mass, 0, 2 * Lf * Caf / Iz, 0;
	SS.B = ts * SS.B;

	SS.C << 0, 1, 0, 0,
		0, 0, 0, 1;

	SS.D = 0;

	return SS;
}

Eigen::Matrix <double, 5, 1> VehicleModel::GetActualStates(Eigen::Matrix <double, 5, 1>& states, double ts) {
	double t_s = ts / 30, t_now = 0;
	while (t_now < ts) {
		states(0, 0) += t_s * (-(2 * Caf + 2 * Car) / (Mass * XVel) * states(0, 0) + (-XVel - (2 * Caf * Lf - 2 * Car * Lr) / (Mass * XVel)) * states(2, 0) + 2 * Caf / Mass * states(4, 0));

		states(1, 0) += t_s * states(2, 0);

		states(2, 0) += t_s * (-(2 * Lf * Caf - 2 * Lr * Car) / (Iz * XVel) * states(0, 0) - (2 * pow(Lf, 2) * Caf + 2 * pow(Lr, 2) * Car) / (Iz * XVel) * states(2, 0) + 2 * Lf * Caf / Iz * states(4, 0));

		states(3, 0) += t_s * (sin(states(1, 0)) * XVel + cos(states(1, 0)) * states(0, 0));

		t_now += t_s;
	}
	return states;
}