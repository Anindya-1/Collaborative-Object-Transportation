#ifndef OPTIMIZER_SQP_HPP
#define OPTIMIZER_SQP_HPP

#include "optimization.h"

alglib::real_1d_array solve_Ik(double x, double y, double z, alglib::real_1d_array& q_prev);

// void forward_kinematics(const alglib::real_1d_array& q, double& x, double& y, double& z);
std::vector<double> forward_kinematics(const alglib::real_1d_array& q);

#endif // OPTIMIZER_SQP_HPP