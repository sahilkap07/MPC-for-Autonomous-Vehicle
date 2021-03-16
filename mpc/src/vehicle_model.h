#pragma once
#include <Eigen/Dense>
#include <vector>

struct StateMatrices
{
	Eigen::Matrix <double, 4, 4> A;
	Eigen::Matrix <double, 4, 1> B;
	Eigen::Matrix <double, 2, 4> C;
	double D;
};

class VehicleModel
{
private:	// mass, moment of Inertia abt c.g, cornering stiffness (front & rear), c.g to front and rear, XVel in body frame
	int Mass, Iz; 
	double Caf, Car, Lf, Lr, XVel; 
	StateMatrices SS;
public:
	VehicleModel();
	VehicleModel(int mass, int iz, double caf, double car, double lf, double lr, double xVel);
	StateMatrices GetStateSpace(double ts);
	Eigen::Matrix <double, 5, 1> GetActualStates(Eigen::Matrix <double, 5, 1>& states, double ts);
};