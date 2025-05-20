import numpy as np
from scipy.optimize import minimize

# Robot link parameters
l2, l3, l4, d1 = 0.13, 0.124, 0.126, 0.077
offset = np.radians(79.38)

# Joint limits and velocity bounds
q_min = np.array([-0.9*np.pi, (-0.57*np.pi), (-0.3*np.pi), -0.57*np.pi])    #(-162, -102, -54, -102)
q_max = np.array([ 0.9*np.pi,  (0.5*np.pi),  (0.44*np.pi),  0.65*np.pi])    #(162, 90, 79.2, 117)
# q_min = np.array([-np.pi, -np.pi, -np.pi, -np.pi])
# q_max = np.array([np.pi, np.pi, np.pi, np.pi])
dq_min = np.array([-1, -1, -1, -1])
dq_max = np.array([ 1,  1,  1,  1])

# Forward kinematics: only position
def forward_kinematics(q):
    q1, q2, q3, q4 = q
    q2_eff = q2 + offset
    q3_eff = q3 - offset  

    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2_eff), np.sin(q2_eff)
    c23, s23 = np.cos(q2_eff + q3_eff), np.sin(q2_eff + q3_eff)
    c234, s234 = np.cos(q2_eff + q3_eff + q4), np.sin(q2_eff + q3_eff + q4)

    x = c1 * (l4*c234 + l3*c23 + l2*c2)
    y = s1 * (l4*c234 + l3*c23 + l2*c2)
    z = d1 + l4*s234 + l3*s23 + l2*s2
    return np.array([x, y, z])

# Cost: position error
def cost_function(q, target_pos):
    pos = forward_kinematics(q)
    return np.linalg.norm(pos - target_pos)**2

# Constraint: q2 + q3 + q4 == 0
def joint_sum_constraint(q):
    return q[1] + q[2] + q[3]

# IK solver with constraint
def solve_ik_with_constraint(x_des, y_des, z_des, q_prev):
    target_pos = np.array([x_des, y_des, z_des])

    # Joint bounds with velocity constraints=
    lower = q_min
    upper = q_max
    bounds = [(lo, hi) for lo, hi in zip(lower, upper)]

    # Define constraint in scipy format
    constraint = {
        'type': 'eq',
        'fun': joint_sum_constraint
    }

    result = minimize(cost_function, q_prev,
                      args=(target_pos,),
                      method='SLSQP',
                      bounds=bounds,
                      constraints=[constraint])
     
    if result.success:
        q_sol = result.x.copy()
        q_sol[1] = (-1*q_sol[1])    # - offset
        q_sol[2] = (-1*q_sol[2])    # + offset
        q_sol[3] = (-1*q_sol[3])
        return q_sol
        # return result.x
    else:
        print("Optimization failed:", result.message)
        return q_prev

# === Test ===
if __name__ == "__main__":
    desired_pos = [0.0, 0.0, 0.2]
    q_initial = np.array([0.0, 0.0, 0.0, 0.0])

    q_sol = solve_ik_with_constraint(*desired_pos, q_initial)
    q_sol_fk = q_sol
    print("Joint solution:", q_sol)
    q_sol_fk[1] = (-1*q_sol[1])     #+ offset
    q_sol_fk[2] = (-1*q_sol[2])     #- offset
    q_sol_fk[3] = (-1*q_sol[3])
    print("Final position:", forward_kinematics(q_sol_fk))
    print(f"{np.degrees(q_sol[0])}, {np.degrees(q_sol[1])}, {np.degrees(q_sol[2])}, {np.degrees(q_sol[3])}")
    print("q2 + q3 + q4 =", q_sol[1] + q_sol[2] + q_sol[3])  # Should be ≈ 0
