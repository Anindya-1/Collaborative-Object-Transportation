#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "optimization.h"

using namespace alglib;

// Link parameters
const double l2 = 0.124;
const double l3 = 0.124;
const double l4 = 0.126;
const double d1 = 0.077;


// Forward kinematics: q → [x, y, z]
std::vector<double> forward_kinematics(const real_1d_array& q)
{
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];

    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2), s2 = sin(q2);
    double c23 = cos(q2 + q3), s23 = sin(q2 + q3);
    double c234 = cos(q2 + q3 + q4), s234 = sin(q2 + q3 + q4);

    double x = c1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double y = s1 * (l4 * c234 + l3 * c23 + l2 * c2);
    double z = d1 + l4 * s234 + l3 * s23 + l2 * s2;

    return {x, y, z};
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

    // fi.setlength(1);
    fi[0] = dx*dx + dy*dy + dz*dz;

    // jac.setlength(1, 4);

    double df_common = l4 * c234 + l3 * c23 + l2 * c2;
    double dfx_dq1 = -s1 * df_common;
    double dfy_dq1 =  c1 * df_common;
    double dfz_dq1 = 0.0;
    jac[0][0] = 2 * (dx * dfx_dq1 + dy * dfy_dq1 + dz * dfz_dq1);

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
}

// Solves IK and returns joint angles
std::vector<double> solve_inverse_kinematics(const std::vector<double>& ee_pos,
    const std::vector<double>& q_prev_vec)
    {
        real_1d_array pEE;
        pEE.setlength(3);
        for (int i = 0; i < 3; ++i) pEE[i] = ee_pos[i];
    
        real_1d_array q_prev;
        q_prev.setlength(4);
        for (int i = 0; i < 4; ++i) q_prev[i] = q_prev_vec[i];
    
        real_1d_array q_min = "[-3.14,-3.14,-3.14,-3.14]";
        real_1d_array q_max = "[3.14, 3.14, 3.14, 3.14]";
        real_1d_array dq_min = "[-1,-1,-1,-1]";
        real_1d_array dq_max = "[1,1,1,1]";
    
        real_1d_array q0 = "[0,0,0,0]";
        real_1d_array scale = "[1,1,1,1]";
        double epsx = 1e-6;
        ae_int_t maxits = 0;
    
        minnlcstate state;
        minnlcreport rep;
        real_1d_array q_result;
    
        minnlccreate(4, q0, state);
        minnlcsetcond(state, epsx, maxits);
        minnlcsetscale(state, scale);
        minnlcsetstpmax(state, 1.0);
    
        real_1d_array lower_bound, upper_bound;
        lower_bound.setlength(4);
        upper_bound.setlength(4);
        for (int i = 0; i < 4; ++i) {
            lower_bound[i] = std::max(q_min[i], q_prev[i] + dq_min[i]);
            upper_bound[i] = std::min(q_max[i], q_prev[i] + dq_max[i]);
        }
    
        minnlcsetbc(state, lower_bound, upper_bound);
        minnlcsetalgosqp(state);
    
        real_1d_array dummy_nl = "[]", dummy_nu = "[]";
        minnlcsetnlc2(state, dummy_nl, dummy_nu);
    
        minnlcoptimize(state, ik_cost_jac, NULL, (void*)(&pEE));
        minnlcresults(state, q_result, rep);
    
        std::vector<double> result(4);
        for (int i = 0; i < 4; ++i) result[i] = q_result[i];
    
        return result;
    }

// g++-11 -I/home/csl-a/alglib-cpp/src /home/csl-a/alglib-cpp/src/*.cpp /home/csl-a/mtp_ws/src/inv_kin_solver/src/optimizer_SQP.cpp -o /home/csl-a/mtp_ws/src/inv_kin_solver/src/optimizer_SQP