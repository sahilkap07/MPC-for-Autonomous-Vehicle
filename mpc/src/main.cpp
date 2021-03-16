#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "trajectory.h"
#include "vehicle_model.h"
#include "mpc.h"
#include "animation.h"	

int main()
{
	int mass = 1500, iz = 3000;
	double ca_f = 19000, ca_r = 33000, lf = 2, lr = 3, x_vel = 20;
	
	const int horizon = 50;
	const double total_time = 10, ts = 0.02;
	const int n_steps = int(total_time / ts) + 1;

	VehicleModel car(mass, iz, ca_f, ca_r, lf, lr, x_vel);
	auto ss = car.GetStateSpace(ts);

	Trajectory track(Maneuver::ObstacleAvoidance, x_vel, total_time, ts);
	auto[x_ref, y_ref, psi_ref] = track.GetTrajectoryPts();

	int step = 0;
	const int n_outs = 2, n_inputs = 1;

	double input = 0;
	Eigen::Matrix <double, 5, 1> states_aug;
	states_aug << 0, 0, 0, y_ref(0) + 10, 0;
	Eigen::MatrixXd ref_traj;
	ref_traj.resize(n_outs * n_steps, 1);
	Eigen::MatrixXd ref_horz;
	ref_horz.resize(n_outs * horizon, 1);
	
	for (int i = 0; i < y_ref.size(); i++) { ref_traj(2 * i, 0) = psi_ref(i); ref_traj(2 * i + 1, 0) = y_ref(i); }

	Eigen::MatrixXd total_states(n_steps, 5);
	total_states.block(0, 0, 1, states_aug.rows()) = states_aug.transpose();
	Eigen::MatrixXd total_psi_opt(n_steps, horizon);
	Eigen::MatrixXd get_psi_opt(horizon, 5 * horizon);
	Eigen::MatrixXd total_Y_opt(n_steps, horizon);
	Eigen::MatrixXd get_Y_opt(horizon, 5 * horizon);
	for (int i = 0; i < horizon; i++) { get_psi_opt(i, 5 * i + 1) = 1; }
	for (int i = 0; i < horizon; i++) { get_Y_opt(i, 5 * i + 3) = 1; }

	Eigen::Matrix <double, n_outs, n_outs> q, s;
	Eigen::Matrix <double, n_inputs, n_inputs> r;
	q << 100, 0,
		0, 1;
	s << 100, 0,
		0, 1;
	r << 100;

	MPC <n_outs, n_inputs> car_controller(q, s, r, ss, horizon);
	car_controller.SetObjMatrix(horizon);
	Eigen::MatrixXd opt_inputs;
	Eigen::MatrixXd opt_states;

	while (step < n_steps - 1) {

		ref_horz = (step + horizon) * n_outs < n_steps * n_outs ? ref_traj.block(step * n_outs, 0, n_outs * horizon, 1) : ref_traj.block(step * n_outs, 0, (n_steps - step) * n_outs, 1);

		car_controller.GetOptVals(states_aug, ref_horz, opt_inputs, opt_states);

		Eigen::MatrixXd psi_opt(ref_horz.rows() / 2, 1);
		psi_opt = get_psi_opt.block(0, 0, ref_horz.rows() / 2, ref_horz.rows() / 2 * 5) * opt_states;
		Eigen::MatrixXd Y_opt(ref_horz.rows() / 2, 1);
		Y_opt = get_Y_opt.block(0, 0, ref_horz.rows() / 2, ref_horz.rows() / 2 * 5) * opt_states;
		total_psi_opt.block(step, 0, 1, ref_horz.rows() / 2) = psi_opt.transpose();
		total_Y_opt.block(step, 0, 1, ref_horz.rows() / 2) = Y_opt.transpose();
		
		states_aug(4, 0) = states_aug(4, 0) + opt_inputs(0, 0);
		//std::cout << states_aug << std::endl;
		if (states_aug(4, 0) < -M_PI / 6) states_aug(4, 0) = -M_PI / 6;
		else if (states_aug(4, 0) > M_PI / 6) states_aug(4, 0) = M_PI / 6;

		states_aug = car.GetActualStates(states_aug, ts);
		total_states.block(step + 1, 0, 1, states_aug.rows()) = states_aug.transpose();

		step += 1;
	}
	//std::cout << sizeof(*(&total_states[0])) << std::endl;
	//std::cout << total_states.size() << ' ' << total_states.cols() << std::endl;
	animate(total_states, x_ref, y_ref, lf, lr);
	//std::cout << typeid(&total_states(0)).name() << std::endl;
	//std::cout << total_states << std::endl;
	std::cin.get();
	return 0;
}