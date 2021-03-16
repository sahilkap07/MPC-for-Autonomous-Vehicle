#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include "vehicle_model.h"
#include "coeff_matrices.h"

Eigen::MatrixXd power(Eigen::MatrixXd& matrix, int p) {

	Eigen::MatrixXd res(matrix.rows(), matrix.cols());
	res.setIdentity();

	while (p > 0) {
		res *= matrix;
		p -= 1;
	}
	return res;
}

template<int N, int M>
class MPC
{
private:
	Eigen::Matrix <double, N, N> Q; // ErrorCost
	Eigen::Matrix <double, N, N> S; // LErrorCost
	Eigen::Matrix <double, M, M> R; // InputCost
	Coefficients Coef;
	StateMatrices SS;
	int Horizon;

public:
	MPC(Eigen::Matrix <double, N, N> q, Eigen::Matrix <double, N, N> s, Eigen::Matrix <double, M, M> r, StateMatrices ss, int horizon) : Q(q), S(s), R(r), SS(ss), Horizon(horizon) {

		Coef.A_aug.resize(5, 5);		// augmented A including 5th state
		Coef.A_aug.block(0, 0, 4, 4) = SS.A;
		Coef.A_aug.block(0, 4, 4, 1) = SS.B;
		Coef.A_aug.row(4) << 0, 0, 0, 0, 1;

		Coef.B_aug.resize(5, 1);		// augmented B
		Coef.B_aug.block(0, 0, 4, 1) = SS.B;
		Coef.B_aug(4, 0) = 1;

		Coef.C_aug.resize(2, 5);		// augmented C
		Coef.C_aug.block(0, 0, 2, 4) = SS.C;
		Coef.C_aug.col(4) << 0, 0;

		Coef.CQC.resize(5, 5);		// intermediate coeff of states
		Coef.CQC = Coef.C_aug.transpose() * Q * Coef.C_aug;

		Coef.CSC.resize(5, 5);		// intermediate coeff of final state
		Coef.CSC = Coef.C_aug.transpose() * S * Coef.C_aug;

		Coef.QC.resize(2, 5);		// intermediate coeff of ref and state
		Coef.QC = Q * Coef.C_aug;

		Coef.SC.resize(2, 5);		// intermediate coeff of final ref and state
		Coef.SC = S * Coef.C_aug;

	}

	// to get the matrices corresponding to final cost function evaluation
	void SetObjMatrix(int horizon) {

		Eigen::MatrixXd Qd(Coef.CQC.rows() * horizon, Coef.CQC.cols() * horizon);
		Qd.setZero();

		Eigen::MatrixXd Td(Coef.QC.rows() * horizon, Coef.QC.cols() * horizon);
		Td.setZero();

		Eigen::MatrixXd Rd(M * horizon, M * horizon);
		Rd.setZero();

		Coef.Cd.resize(Coef.B_aug.rows() * horizon, Coef.B_aug.cols() * horizon);
		Coef.Cd.setZero();

		Coef.Ad.resize(Coef.A_aug.rows() * horizon, Coef.A_aug.cols());
		Coef.Ad.setZero();

		for (int i = 0; i < horizon; i++) {

			Qd.block(i * Coef.CQC.rows(), i * Coef.CQC.cols(), Coef.CQC.rows(), Coef.CQC.cols()) = i < horizon - 1 ? Coef.CQC : Coef.CSC;
			Td.block(i * Coef.QC.rows(), i * Coef.QC.cols(), Coef.QC.rows(), Coef.QC.cols()) = i < horizon - 1 ? Coef.QC : Coef.SC;
			Rd.block(i * M, i * M, M, M) = R;

			for (int j = 0; j < i + 1; j++) {
				Coef.Cd.block(i * Coef.B_aug.rows(), j, Coef.B_aug.rows(), 1) = power(Coef.A_aug, i - j) * Coef.B_aug;
			}

			Coef.Ad.block(i * Coef.A_aug.rows(), 0, Coef.A_aug.rows(), Coef.A_aug.cols()) = power(Coef.A_aug, i + 1);
		}

		Eigen::MatrixXd Hd(Rd.rows(), Rd.cols());
		Hd = Coef.Cd.transpose() * Qd * Coef.Cd + Rd;

		Eigen::MatrixXd Fd(Coef.Ad.cols() + Td.rows(), Coef.Cd.cols());
		Fd.block(0, 0, Coef.Ad.cols(), Coef.Cd.cols()) = Coef.Ad.transpose() * Qd * Coef.Cd;
		Fd.block(Coef.Ad.cols(), 0, Td.rows(), Coef.Cd.cols()) = -Td * Coef.Cd;

		Coef.ObjMatrix = -Hd.inverse() * Fd.transpose();
	}

	void GetOptVals(Eigen::Matrix <double, 5, 1>& states_aug, Eigen::MatrixXd& reference, Eigen::MatrixXd& OptInputs, Eigen::MatrixXd& OptStates) {

		if (reference.rows()/2 != Horizon)
			SetObjMatrix(reference.rows()/2);

		Eigen::MatrixXd states_ref(states_aug.rows() + reference.rows(), 1);
		states_ref << states_aug, reference;
		OptInputs = Coef.ObjMatrix * states_ref;
		OptStates = Coef.Cd * OptInputs + Coef.Ad * states_aug;
	}

};