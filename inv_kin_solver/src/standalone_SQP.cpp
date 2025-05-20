#include "optimization.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>


using namespace alglib;

// Link parameters
const double l2 = 0.13;
const double l3 = 0.124;
const double l4 = 0.126;
const double d1 = 0.077;

// Joint positions and limits
real_1d_array q_min = "[-3.14,-2*3.14,-2*3.14,-3.14]";
real_1d_array q_max = "[3.14, 2*3.14, 2*3.14, 3.14]";
real_1d_array dq_min = "[-2,-2,-2,-2]";
real_1d_array dq_max = "[2,2,2,2]";

void rotationMatrixToRPY(const std::vector<std::vector<double>>& R, double& roll, double& pitch, double& yaw)
{
    if (R.size() != 3 || R[0].size() != 3 || R[1].size() != 3 || R[2].size() != 3) {
        std::cerr << "Invalid rotation matrix size." << std::endl;
        roll = pitch = yaw = 0;
        return;
    }

    pitch = std::asin(-R[2][0]);
    if (std::abs(R[2][0]) < 0.99999) {
        roll  = std::atan2(R[2][1], R[2][2]);
        yaw   = std::atan2(R[1][0], R[0][0]);
    } else {
        // Gimbal lock case
        roll  = 0;
        yaw   = std::atan2(-R[0][1], R[1][1]);
    }
}

// Forward kinematics: q → [x, y, z, R, P, Y]
std::vector<double> forward_kinematics(const real_1d_array& q)
{
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];

    // Apply the offsets
    const double offset = 79.38 * M_PI / 180.0;
    double q2_eff = q2 + offset;
    double q3_eff = q3 - offset;

    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2_eff), s2 = sin(q2_eff);
    double c23 = cos(q2_eff + q3_eff), s23 = sin(q2_eff + q3_eff);
    double c234 = cos(q2_eff + q3_eff + q4), s234 = sin(q2_eff + q3_eff + q4);

    // EE position
    double x = c1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double y = s1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double z = d1 + l4 * s234 + l3 * s23 + l2 * s2;

    // EE orientation
    std::vector<std::vector<double>> rot_mat(3, std::vector<double>(3));
    rot_mat[0][0] = c1 * c234;  rot_mat[0][1] = -c1 * s234; rot_mat[0][2] = s1;
    rot_mat[1][0] = s1 * c234;  rot_mat[1][1] = -s1 * s234; rot_mat[1][2] = c1;
    rot_mat[2][0] = s234;       rot_mat[2][1] = c234;       rot_mat[2][2] = 0;

    double roll, pitch, yaw;
    rotationMatrixToRPY(rot_mat, roll, pitch, yaw);

    return {x, y, z, roll, pitch, yaw};
}

// Cost function and Jacobian
void ik_cost_jac(const real_1d_array &q, real_1d_array &fi, real_2d_array &jac, void *ptr)
{
    real_1d_array* pEE = static_cast<real_1d_array*>(ptr);
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];

    // Define offsets in radians
    const double offset = 79.38 * M_PI / 180.0;
    double q2_eff = q2 + offset;
    double q3_eff = q3 - offset;

    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2_eff), s2 = sin(q2_eff);
    double c23 = cos(q2_eff + q3_eff), s23 = sin(q2_eff + q3_eff);
    double c234 = cos(q2_eff + q3_eff + q4), s234 = sin(q2_eff + q3_eff + q4);

    double fx = c1 * (l4*c234 + l3*c23 + l2*c2);
    double fy = s1 * (l4*c234 + l3*c23 + l2*c2);
    double fz = d1 + l4*s234 + l3*s23 + l2*s2;

    double dx = fx - (*pEE)[0];
    double dy = fy - (*pEE)[1];
    double dz = fz - (*pEE)[2];

    fi[0] = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Constraint 1: q2 + q3 + q4 = 0
    // fi[1] = q2 + q3 + q4;

    double df_common = l4 * c234 + l3 * c23 + l2 * c2;
    double dfx_dq1 = -s1 * df_common;
    double dfy_dq1 =  c1 * df_common;
    jac[0][0] = 2 * (dx * dfx_dq1 + dy * dfy_dq1);

    // Derivatives w.r.t q2 and q3 must use chain rule due to offset
    double ds2_dq2 = cos(q2_eff);
    double dc2_dq2 = -sin(q2_eff);

    double ds23_dq2 = cos(q2_eff + q3_eff);
    double dc23_dq2 = -sin(q2_eff + q3_eff);

    double ds234_dq2 = cos(q2_eff + q3_eff + q4);
    double dc234_dq2 = -sin(q2_eff + q3_eff + q4);

    double dfx_dq2 = c1 * (-l4 * dc234_dq2 - l3 * dc23_dq2 - l2 * dc2_dq2);
    double dfy_dq2 = s1 * (-l4 * dc234_dq2 - l3 * dc23_dq2 - l2 * dc2_dq2);
    double dfz_dq2 = l4 * ds234_dq2 + l3 * ds23_dq2 + l2 * ds2_dq2;
    jac[0][1] = 2 * (dx * dfx_dq2 + dy * dfy_dq2 + dz * dfz_dq2);

    double ds23_dq3 = cos(q2_eff + q3_eff);
    double dc23_dq3 = -sin(q2_eff + q3_eff);

    double ds234_dq3 = cos(q2_eff + q3_eff + q4);
    double dc234_dq3 = -sin(q2_eff + q3_eff + q4);

    double dfx_dq3 = c1 * (-l4 * dc234_dq3 - l3 * dc23_dq3);
    double dfy_dq3 = s1 * (-l4 * dc234_dq3 - l3 * dc23_dq3);
    double dfz_dq3 = l4 * ds234_dq3 + l3 * ds23_dq3;
    jac[0][2] = 2 * (dx * dfx_dq3 + dy * dfy_dq3 + dz * dfz_dq3);

    double dfx_dq4 = c1 * (-l4 * s234);
    double dfy_dq4 = s1 * (-l4 * s234);
    double dfz_dq4 = l4 * c234;
    jac[0][3] = 2 * (dx * dfx_dq4 + dy * dfy_dq4 + dz * dfz_dq4);

    // Constraint 1 derivatives
    // jac[1][0] = 0; jac[1][1] = 1; jac[1][2] = 1; jac[1][3] = 1;
}


// Solve IK
real_1d_array solve_Ik(double x, double y, double z, real_1d_array& q_prev)
{   
    try
        {
        real_1d_array q0 = q_prev;
        real_1d_array scale = "[1,1,1,1]";
        real_1d_array pEE;
        pEE.setlength(3);
        pEE[0] = x;
        pEE[1] = y;
        pEE[2] = z;

        minnlcstate state;
        minnlcreport rep;
        real_1d_array q_result;

        minnlccreate(4, q0, state);
        minnlcsetcond(state, 1e-4, 1000);      //stoping conditions
        minnlcsetscale(state, scale);
        minnlcsetstpmax(state, 3.0);

        real_1d_array lower_bound = "[0,0,0,0]";
        real_1d_array upper_bound = "[0,0,0,0]";
        for (int i = 0; i < 4; ++i) {
            // lower_bound[i] = std::max(q_min[i], q_prev[i] + dq_min[i]);
            // upper_bound[i] = std::min(q_max[i], q_prev[i] + dq_max[i]);
            lower_bound[i] = q_min[i];
            upper_bound[i] = q_max[i];
        }

        minnlcsetbc(state, lower_bound, upper_bound);
        minnlcsetalgosqp(state);

        real_1d_array dummy_nl = "[]", dummy_nu = "[]";
        minnlcsetnlc2(state, dummy_nl, dummy_nu);

        // real_1d_array eq_constr_lb = "[0.0]";
        // real_1d_array eq_constr_ub = "[0.0]";
        // minnlcsetnlc2(state, eq_constr_lb, eq_constr_ub);


        minnlcoptimize(state, ik_cost_jac, NULL, (void*)(&pEE));
        minnlcresults(state, q_result, rep);
        std::cout << "Termination type: " << rep.terminationtype << std::endl;


        return q_result;
    }
    catch(alglib::ap_error alglib_exception)
    {
        // printf("ALGLIB exception with message '%s'\n", alglib_exception.msg.c_str());
        std::cerr << "ALGLIB exception: " << alglib_exception.msg.c_str() << std::endl;
        return q_prev;
    }
}

int main(){
    std::vector<double> ee_pos = {0.0, 0.0, 0.35};
    std::vector<double> pred_ee_pos;

    std::cout << "desired EE pose will be: ";
    for(auto co_ord : ee_pos){
        std::cout << co_ord << ", ";  
    }
    std::cout << std::endl;

    real_1d_array q_prev = "[0, 0, 0, 0]";
    real_1d_array q_new = solve_Ik(ee_pos[0], ee_pos[1], ee_pos[2], q_prev);
    std::cout << "\nOptimized joint angles:\n";
    for(int i = 0; i < 4; ++i){
        std::cout << q_new[i] << " ";
    }
    std::cout << std::endl;
    // real_1d_array q_new = "[0.5, 0.5, 0, -0.5]";
    pred_ee_pos = forward_kinematics(q_new);

    std::cout << "EE pose will be: ";
    for(auto co_ord : pred_ee_pos){
        std::cout << co_ord << ", ";  
    }
    std::cout << std::endl;

    return 0;
}


// g++-11 -I/home/csl-a/alglib-cpp/src /home/csl-a/alglib-cpp/src/*.cpp /home/csl-a/mtp_ws/src/inv_kin_solver/src/standalone_SQP.cpp -o /home/csl-a/mtp_ws/src/inv_kin_solver/src/standalone_SQP