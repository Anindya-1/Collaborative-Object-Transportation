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

# Transform EE position from base frame to world frame using base pose (x, y, theta)
def ee_pos_world_frame(q, base_pos, base_height):
    ee_pos_local = forward_kinematics(q)
    x_b, y_b, theta_b = base_pos

    # Rotation matrix around z-axis
    R = np.array([
        [np.cos(theta_b), -np.sin(theta_b), 0],
        [np.sin(theta_b),  np.cos(theta_b), 0],
        [0,               0,               1]
    ])
    ee_pos_world = R @ ee_pos_local + np.array([x_b, y_b, base_height])
    return ee_pos_world

# Cost: position error
def cost_function(q, target_pos_world, base_pos, base_height):
    ee_world = ee_pos_world_frame(q, base_pos, base_height)
    return np.linalg.norm(ee_world - target_pos_world)**2

# Constraint: q2 + q3 + q4 == 0
def joint_sum_constraint(q):
    return q[1] + q[2] + q[3]

# Constraint: distance between current FK and fixed other EE pos
def distance_constraint(q, fixed_pos_world, d_fixed, base_pos, base_height):
    curr_pos_world = ee_pos_world_frame(q, base_pos, base_height)
    return np.linalg.norm(curr_pos_world - fixed_pos_world) - d_fixed

# IK solver with constraint
def solve_ik_with_constraint(x_des, y_des, z_des, q_prev, fixed_pos_world, d_fixed, base_pos, base_height):
    target_pos_world = np.array([x_des, y_des, z_des])

    # Joint bounds with velocity constraints=
    lower = q_min
    upper = q_max
    bounds = [(lo, hi) for lo, hi in zip(lower, upper)]

    # Define constraint in scipy format
    constraints = [
        {'type': 'eq', 'fun': joint_sum_constraint},
        {'type': 'eq', 'fun': lambda q: distance_constraint(q, fixed_pos_world, d_fixed, base_pos, base_height)},
        # {'type': 'ineq', 'fun': lambda q: 0.01 - abs(distance_constraint(q, fixed_pos_world, d_fixed, base_pos, base_height))}

        ]

    result = minimize(cost_function, q_prev,
                      args=(target_pos_world, base_pos, base_height),
                      method='SLSQP',
                      bounds=bounds,
                      constraints=constraints)
     
    if result.success:
        q_sol = result.x.copy()
        q_sol[1] = (-1*q_sol[1])   
        q_sol[2] = (-1*q_sol[2])    
        q_sol[3] = (-1*q_sol[3])
        return q_sol
    else:
        print("Optimization failed:", result.message)
        return q_prev

# === Test ===
if __name__ == "__main__":
    base_pose = np.array([0.0, 0.0, 0.0])
    base_height = 0.1  # example base height of robot base frame relative to world frame z=0

    desired_pos_world = np.array([0.2, 0.0, 0.2])
    other_ee_pos_world = np.array([0.1, 0.0, 0.2])
    d_fixed_val = np.linalg.norm(desired_pos_world - other_ee_pos_world)

    q_initial = np.array([0.0, 0.0, 0.0, 0.0])

    q_solution = solve_ik_with_constraint(*desired_pos_world, q_initial, other_ee_pos_world, d_fixed_val, base_pose, base_height)

    print("Joint solution:", q_solution)

    q_fk = q_solution.copy()
    q_fk[1] = -q_fk[1]
    q_fk[2] = -q_fk[2]
    q_fk[3] = -q_fk[3]

    final_pos_world = ee_pos_world_frame(q_fk, base_pose, base_height)
    print("Final EE position (world):", final_pos_world)
    print("Distance to other EE:", np.linalg.norm(final_pos_world - other_ee_pos_world))
