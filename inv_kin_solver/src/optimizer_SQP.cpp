#include "optimizer_SQP.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace alglib;

// Link parameters
const double l2 = 0.13;
const double l3 = 0.124;
const double l4 = 0.126;
const double d1 = 0.077;

// Joint positions and limits
// real_1d_array q_min = "[-3.14*0.9, (-3.14*0.57), (-3.14*0.3), -3.14*0.57]";
// real_1d_array q_max = "[ 3.14*0.9,  (3.14*0.5),  (3.14*0.44),  3.14*0.65]";
real_1d_array q_min = "[-3.14,-3.14,-3.14,-3.14]";
real_1d_array q_max = "[ 3.14, 3.14, 3.14, 3.14]";
real_1d_array dq_min = "[-1,-1,-1,-1]";
real_1d_array dq_max = "[1,1,1,1]";

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

    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2), s2 = sin(q2);
    double c23 = cos(q2 + q3), s23 = sin(q2 + q3);
    double c234 = cos(q2 + q3 + q4), s234 = sin(q2 + q3 + q4);

    //EE position
    double x = c1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double y = s1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double z = d1 + l4 * s234 + l3 * s23 + l2 * s2;

    //EE orientation
    std::vector<std::vector<double>> rot_mat(3, std::vector<double>(3));
    rot_mat[0][0] = c1*c234; rot_mat[0][1] = -1*c1*s234; rot_mat[0][2] = s1;
    rot_mat[1][0] = s1*c234; rot_mat[1][1] = -1*s1*s234; rot_mat[1][2] = c1;
    rot_mat[2][0] = s234; rot_mat[2][1] = c234; rot_mat[2][2] = 0;

    double roll, pitch, yaw;
    rotationMatrixToRPY(rot_mat, roll, pitch, yaw);

    return {x, y, z, roll, pitch, yaw};
}


// Cost function and Jacobian
void ik_cost_jac(const real_1d_array &q, real_1d_array &fi, real_2d_array &jac, void *ptr)
{
    real_1d_array* pEE = static_cast<real_1d_array*>(ptr);
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];

    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2), s2 = sin(q2);
    double c23 = cos(q2+q3), s23 = sin(q2+q3);
    double c234 = cos(q2+q3+q4), s234 = sin(q2+q3+q4);

    double fx = c1 * (l4*c234 + l3*c23 + l2*c2);
    double fy = s1 * (l4*c234 + l3*c23 + l2*c2);
    double fz = d1 + l4*s234 + l3*s23 + l2*s2;

    double dx = fx - (*pEE)[0];
    double dy = fy - (*pEE)[1];
    double dz = fz - (*pEE)[2];

    fi[0] = dx*dx + dy*dy + dz*dz;
    
    // Constraint 1: q2 + q3 + q4 = 0
    fi[1] = q2 + q3 + q4;

    double df_common = l4 * c234 + l3 * c23 + l2 * c2;
    double dfx_dq1 = -s1 * df_common;
    double dfy_dq1 =  c1 * df_common;
    jac[0][0] = 2 * (dx * dfx_dq1 + dy * dfy_dq1);

    double dfx_dq2 = c1 * (-l4 * s234 - l3 * s23 - l2 * s2);
    double dfy_dq2 = s1 * (-l4 * s234 - l3 * s23 - l2 * s2);
    double dfz_dq2 = l4 * c234 + l3 * c23 + l2 * c2;
    jac[0][1] = 2 * (dx * dfx_dq2 + dy * dfy_dq2 + dz * dfz_dq2);

    double dfx_dq3 = c1 * (-l4 * s234 - l3 * s23);
    double dfy_dq3 = s1 * (-l4 * s234 - l3 * s23);
    double dfz_dq3 = l4 * c234 + l3 * c23;
    jac[0][2] = 2 * (dx * dfx_dq3 + dy * dfy_dq3 + dz * dfz_dq3);

    double dfx_dq4 = c1 * (-l4 * s234);
    double dfy_dq4 = s1 * (-l4 * s234);
    double dfz_dq4 = l4 * c234;
    jac[0][3] = 2 * (dx * dfx_dq4 + dy * dfy_dq4 + dz * dfz_dq4);

    // Constraint 1: q2 + q3 + q4 = 0 ⇒ ∂/∂q2 = ∂/∂q3 = ∂/∂q4 = 1
    jac[1][0] = 0; jac[1][1] = 1; jac[1][2] = 1; jac[1][3] = 1;
}


// Solve IK
real_1d_array solve_Ik(double x, double y, double z, real_1d_array& q_prev)
{
    if (q_prev.length() != 4) {
        std::cerr << "q_prev must have length 4" << std::endl;
        return q_prev;
    }

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
    minnlcsetcond(state, 1e-6, 0);      //stoping conditions
    minnlcsetscale(state, scale);
    minnlcsetstpmax(state, 1.0);

    real_1d_array lower_bound = "[0,0,0,0]";
    real_1d_array upper_bound = "[0,0,0,0]";
    for (int i = 0; i < 4; ++i) {
        lower_bound[i] = q_min[i];
        upper_bound[i] = q_max[i];

        if (lower_bound[i] > upper_bound[i]) {
            std::cerr << "Invalid bounds for joint " << i << ": "
                        << "lower = " << lower_bound[i]
                        << ", upper = " << upper_bound[i] << std::endl;
            return q_prev;
        }
    }

    minnlcsetbc(state, lower_bound, upper_bound);
    minnlcsetalgosqp(state);

    // real_1d_array dummy_nl = "[]", dummy_nu = "[]";
    // minnlcsetnlc2(state, dummy_nl, dummy_nu);

    real_1d_array eq_constr_lb = "[0.0]";
    real_1d_array eq_constr_ub = "[0.0]";
    minnlcsetnlc2(state, eq_constr_lb, eq_constr_ub);

    // minnlcoptimize(state, ik_cost_jac, NULL, (void*)(&pEE));
    try {
        minnlcoptimize(state, ik_cost_jac, NULL, (void*)(&pEE));
    } catch (alglib::ap_error& e) {
        std::cerr << "ALGLIB ERROR: " << e.msg << std::endl;
        return q_prev;
    }
    minnlcresults(state, q_result, rep);

    return q_result;
}
