#pragma once
#include <Eigen/Dense>

struct Coefficients
{
	Eigen::MatrixXd A_aug;
	Eigen::MatrixXd B_aug;
	Eigen::MatrixXd C_aug;
	Eigen::MatrixXd CQC;
	Eigen::MatrixXd CSC;
	Eigen::MatrixXd SC;
	Eigen::MatrixXd QC;
	Eigen::MatrixXd Ad;
	Eigen::MatrixXd Cd;
	Eigen::MatrixXd ObjMatrix;
};