#pragma once
#include "engine.h"
#include <Eigen/Dense>
#include <iostream>

#pragma comment ( lib, "libmat.lib" )
#pragma comment ( lib, "libmx.lib" )
#pragma comment ( lib, "libmex.lib" )
#pragma comment ( lib, "libeng.lib" )

void animate(Eigen::MatrixXd& total_states, Eigen::ArrayXd& x_ref, Eigen::ArrayXd& y_ref, double& lf, double& lr);