
import numpy as np
import cvxpy as cp
# from control_scripts.add_obstacles import add_obstacles
# from dynamic_obstacles import obstacle_avoidance_constraint, DynamicObstacle

# # Dummy dynamic obstacles
# obstacle1 = DynamicObstacle(position=np.array([5.0, 0.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)
# obstacle2 = DynamicObstacle(position=np.array([0.0, 3.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)

# List of dynamic obstacles
# dynamic_obstacles = [obstacle1, obstacle2]

def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity, max_acceleration, goal):
    # Pad the waypoint with zeros to match 6D state
    waypoint_padded = np.concatenate((waypoint, np.zeros(3)))

    if waypoint[0] == goal[0] and waypoint[1] == goal[1] and waypoint[2] == goal[2]:
        Q = np.diag([100, 100, 100, 100, 100, 100])
        
    # Initialize state (6D) and control (3D) variables
    x = cp.Variable((6, horizon + 1))
    u = cp.Variable((3, horizon))
    safety_margin = 0.5
    cost = 0
    # Initial state constraint, make sure the drone starts at the initial position
    constraints = [x[:, 0] == x_init]

    # MPC problem formulation
    for t in range(horizon):
        cost += cp.quad_form(x[:, t] - waypoint_padded, Q)
        if t < horizon - 1:
            cost += cp.quad_form(u[:, t], R)
            constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t]]

            # # Run function obstacle_avoidance_constraint to add constraints of the dynamical obstacles to the MPC problem
            # obstacle_constraint_t = obstacle_avoidance_constraint(x[:, t], u[:, t], dynamic_obstacles, safety_margin)
            # constraints += obstacle_constraint_t

            constraints += [cp.abs(x[3, t]) <= max_velocity]  # Velocity in x
            constraints += [cp.abs(x[4, t]) <= max_velocity]  # Velocity in y
            constraints += [cp.abs(x[5, t]) <= max_velocity]  # Velocity in z

            constraints += [cp.abs(u[0, t]) <= max_acceleration]
            constraints += [cp.abs(u[1, t]) <= max_acceleration]
            constraints += [cp.abs(u[2, t]) <= max_acceleration]
            # print(constraints)
            

    # Penalize terminal state
    cost += cp.quad_form(x[:, horizon] - waypoint_padded, Q)

    # Define and solve the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve()
    # print("Problem status", problem.status)
    if problem.status != cp.OPTIMAL:
        raise Exception("The MPC problem is infeasible.")

    return u[:, 0].value, x[:, 1].value

def is_waypoint_reached(current_position, waypoint, tolerance):
    """Check if the current position is within a certain distance of the waypoint."""
    return np.linalg.norm(np.array(current_position) - np.array(waypoint[:3])) < tolerance

