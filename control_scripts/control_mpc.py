
import numpy as np
import cvxpy as cp
import pybullet as p

# Function to control the drone with MPC from waypoint to waypoint
def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity,
                       max_acceleration, goal, condition_for_avoiding_obstacle_is_true):
    # Pad the waypoint with zeros to match 6D state
    waypoint_padded = np.concatenate((waypoint, np.zeros(3)))

    # Change the cost function when the waypoint == goal
    if waypoint[0] == goal[0] and waypoint[1] == goal[1] and waypoint[2] == goal[2]:
        Q = np.diag([100, 100, 100, 100, 100, 100])
        
    # Initialize state (6D) and control (3D) variables
    x = cp.Variable((6, horizon + 1))
    u = cp.Variable((3, horizon))
    cost = 0

    # Initial state constraint, make sure the drone starts at the initial state or current state
    constraints = [x[:, 0] == x_init]

    # MPC problem formulation
    for t in range(horizon):
        # Change in the cost function when a dynamic obstacle is detected, heavily penalized. 
        if condition_for_avoiding_obstacle_is_true:
            Q_obstacle_vel = np.diag([10000, 10000, 10000])
            Q_obstacle_pos = np.diag([10000, 10000, 10000])
            cost += cp.quad_form(x[3:, t] - np.array([0, 0,0]), Q_obstacle_vel)
            cost += cp.quad_form(x[:3, t] - x[:3, horizon], Q_obstacle_pos)    
        else:
            cost += cp.quad_form(x[:, t] - waypoint_padded, Q) # Penalize the distance from the waypoint
        if t < horizon - 1: 
            cost += cp.quad_form(u[:, t], R) # Penalize the control input
            constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t]] # System dynamics constraints
            constraints += [cp.abs(x[3:6, t]) <= max_velocity]  # Velocity constraints
            constraints += [cp.abs(u[:, t]) <= max_acceleration] # Acceleration constraints

    # Penalize terminal state
    if condition_for_avoiding_obstacle_is_true:
        Q_obstacle_vel = np.diag([100, 100])
        cost += cp.quad_form(x[4:, horizon] - np.array([0, 0]), Q_obstacle_vel)
    else:
        cost += cp.quad_form(x[:, horizon] - waypoint_padded, Q)

    #### Debug printing ####
    # if condition_for_avoiding_obstacle_is_true:
    #     print("Obstacle avoidance is on")

    # Define and solve the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve()
    
    # print("Problem status", problem.status)
    if problem.status != cp.OPTIMAL:
        raise Exception("The MPC problem is infeasible.")

    return u[:, 0].value # Return the control input

# Function to check if the current position is within a certain distance of the waypoint
def is_waypoint_reached(current_position, waypoint, tolerance):
    """Check if the current position is within a certain distance of the waypoint."""
    return np.linalg.norm(np.array(current_position) - np.array(waypoint[:3])) < tolerance

# Function to check if the drone is close to a dynamic obstacle
# def condition_for_avoiding_obstacle(drone_position, obstacle_ids, safety_margin): 
#     min_distance = float('inf')
#     for obstacle_id in obstacle_ids:
#         obstacle_position, _ = p.getBasePositionAndOrientation(obstacle_id) # Get the position of the obstacle
#         if obstacle_position[1] > drone_position[1]: #Only look at the columns in front of the quadrotor, because the quadrotor won't fly backwards
#             distance = np.linalg.norm(np.array(drone_position[:2]) - np.array(obstacle_position[:2])) # Calculate the eucledian distance between the quadrotor and the dynamic obstacle in the x,y plane
#             if distance < min_distance:
#                 min_distance = distance
#     if min_distance < safety_margin: # If the distance is smaller than the safety margin, return True (turn on obstacle avoidance)
#         return True
#     else:
#         return False

def condition_for_avoiding_obstacle(drone_position, obstacle_ids, safety_margin, previous_min_distance):
    min_distance = float('inf')
    for obstacle_id in obstacle_ids:
        obstacle_position, _ = p.getBasePositionAndOrientation(obstacle_id)
        if obstacle_position[1] > drone_position[1]:
            distance = np.linalg.norm(np.array(drone_position[:2]) - np.array(obstacle_position[:2]))
            if distance < min_distance:
                min_distance = distance

    if previous_min_distance is not None and min_distance > previous_min_distance:
        return (False, min_distance)
    elif min_distance < safety_margin:
        return (True, min_distance)
    else:
        return (False, min_distance)