#include "animation.h"

void animate(Eigen::MatrixXd& total_states, Eigen::ArrayXd& x_ref, Eigen::ArrayXd& y_ref, double& lf, double& lr) {

	Engine* eng = engOpen(NULL);

	/*Eigen::MatrixXd tt(10, 1);
	tt.setZero();*/
	double* states_ptr = &total_states(0);
	double* x_refptr = &x_ref(0);
	double* y_refptr = &y_ref(0);
	mxDouble* lf_ptr = &lf;
	mxDouble* lr_ptr = &lr;
	//std::cout << sizeof(total_states) << ' ' << sizeof(double) << std::endl;
	
	//mxArray* TotalStates = mxCreateDoubleMatrix(10, 1, mxREAL);
	mxArray* TotalStates = mxCreateDoubleMatrix(total_states.rows(), total_states.cols(), mxREAL);
	mxArray* XRef = mxCreateDoubleMatrix(x_ref.size(), 1, mxREAL);
	mxArray* YRef = mxCreateDoubleMatrix(y_ref.size(), 1, mxREAL);

	memcpy((void*)mxGetDoubles(TotalStates), (void*)states_ptr, sizeof(double) * (total_states.rows() * total_states.cols()));
	memcpy((void*)mxGetDoubles(XRef), (void*)x_refptr, sizeof(double) * (x_ref.size()));
	memcpy((void*)mxGetDoubles(YRef), (void*)y_refptr, sizeof(double) * (y_ref.size()));
	//memcpy((void *)mxGetDoubles(TotalStates), (void *)states_ptr, sizeof(double) * (10)); 

	engPutVariable(eng, "states", TotalStates);
	engPutVariable(eng, "x_ref", XRef);
	engPutVariable(eng, "y_ref", YRef);
	//engPutVariable(eng, "lf", lf_ptr);
	//engPutVariable(eng, "lr", lr_ptr);

	engEvalString(eng, "addpath(genpath('C:\\Users\\sahil\\OneDrive\\Documents\\Visual Studio 2019\\Projects\\mpc\\mpc'))");

	engEvalString(eng, "Animate");

	//engClose(eng);
}