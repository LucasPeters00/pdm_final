
import numpy as np
import cvxpy as cp
import pybullet as p
# from control_scripts.add_obstacles import add_obstacles
# from dynamic_obstacles import obstacle_avoidance_constraint, DynamicObstacle

# # Dummy dynamic obstacles
# obstacle1 = DynamicObstacle(position=np.array([5.0, 0.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)
# obstacle2 = DynamicObstacle(position=np.array([0.0, 3.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)

# List of dynamic obstacles
# dynamic_obstacles = [obstacle1, obstacle2]

def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity,
                       max_acceleration, goal, condition_for_avoiding_obstacle_is_true):
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

        if condition_for_avoiding_obstacle_is_true:
            Q_obstacle_vel = np.diag([10000, 10000])
            Q_obstacle_pos = np.diag([10000, 10000, 10000])
            cost += cp.quad_form(x[4:, t] - np.array([0, 0]), Q_obstacle_vel)
            cost += cp.quad_form(x[:3, t] - x[:3, horizon], Q_obstacle_pos)    

        else:
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

    # Penalize terminal state
    if condition_for_avoiding_obstacle_is_true:
        Q_obstacle_vel = np.diag([100, 100])
        cost += cp.quad_form(x[4:, horizon] - np.array([0, 0]), Q_obstacle_vel)
    else:
        cost += cp.quad_form(x[:, horizon] - waypoint_padded, Q)

    if condition_for_avoiding_obstacle_is_true:
        print("Obstacle avoidance is on")

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

def condition_for_avoiding_obstacle(drone_position, obstacle_ids, safety_margin):
    min_distance = float('inf')

    for obstacle_id in obstacle_ids:
        obstacle_position, _ = p.getBasePositionAndOrientation(obstacle_id)
        #Only look at the columns in front of the drone
        if obstacle_position[1] > drone_position[1]:
            distance = np.linalg.norm(np.array(drone_position[:2]) - np.array(obstacle_position[:2]))
            #Get the closest column
            if distance < min_distance:
                min_distance = distance

            # print("min_distance", min_distance)

    if min_distance < safety_margin:
        return True
    else:
        return False
    """Check if the drone is within a certain distance of the obstacle."""

    # if np.linalg.norm(np.array(current_position) - np.array(waypoint[:3])) < tolerance:
    #     return True
    # else:
    #     return False

