import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
# Import the Drone class from its script file
from drone import Drone

import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Initialize the simulation
p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

# Load the drone into the simulation
drone_model = "urdf_files/cf2x.urdf"
start_position = np.array([0, 0, 0.25 + 0.5])
drone_mass = 0.027 * 10**0
max_velocity = 8.333  # ms^-1
max_acceleration = 22.07  # ms^-2
horizon = 10
dt = 0.01
Q = np.diag([100, 100, 100, 0, 0, 0])
R = np.diag([0.001, 0.001, 0.001])
my_drone = Drone(drone_model, start_position, drone_mass, dt)
tolerance = 0.1
safety_margin = 0.7

# Define the start and goal positions 
start = np.array([0, 0, 0.25 + 0.5])
goal = np.array([0, 1, 0.25 + 0.5])

path = [[0,0,.75],
    [0.1, 0.0, 0.75], [0.2, 0.0, 0.75], [0.3, 0.0, 0.75], [0.4, 0.0, 0.75],
    [0.5, 0.0, 0.75], [0.6, 0.0, 0.75], [0.7, 0.0, 0.75], [0.8, 0.0, 0.75],
    [0.9, 0.0, 0.75], [1, 0.0, 0.75], [1.1, 0.0, 0.75], [1.2, 0.0, 0.75]
]

def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity,
                       max_acceleration, goal, condition_for_avoiding_obstacle_is_true):
    # Pad the waypoint with zeros to match 6D state
    waypoint_padded = np.concatenate((waypoint, np.zeros(3)))

    if waypoint[0] == goal[0] and waypoint[1] == goal[1] and waypoint[2] == goal[2]:
        Q = np.diag([100, 100, 100, 100, 100, 100])
        
    # Initialize state (6D) and control (3D) variables
    x = cp.Variable((6, horizon + 1))
    u = cp.Variable((3, horizon))
    safety_margin = 0.8
    cost = 0
    # Initial state constraint, make sure the drone starts at the initial position
    constraints = [x[:, 0] == x_init]

    # MPC problem formulation
    for t in range(horizon):

        if condition_for_avoiding_obstacle_is_true:
            Q_obstacle_vel = np.diag([10000, 10000, 10000])
            Q_obstacle_pos = np.diag([10000, 10000, 10000])
            cost += cp.quad_form(x[3:, t] - np.array([0, 0,0]), Q_obstacle_vel)
            cost += cp.quad_form(x[:3, t] - x[:3, horizon], Q_obstacle_pos)    

        else:
            cost += cp.quad_form(x[:, t] - waypoint_padded, Q)

        if t < horizon - 1:
            cost += cp.quad_form(u[:, t], R)
            constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t]]
            constraints += [cp.abs(x[3:6, t]) <= max_velocity]  # Velocity constraints
            constraints += [cp.abs(u[:, t]) <= max_acceleration] # Acceleration constraints

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

def condition_for_avoiding_obstacle(waypoint):   

    if waypoint[0] > 1.0:
        return True
    else:
        return False

x_positions = []
x_velocities = []
x_accelerations = []
time_stamps = []
stop_time = None
velocity_threshold = 0.01
acceleration_threshold = 0.001
stop_drone = False

# Control (MPC) Loop
for waypoint in path:
    if stop_drone:
        break
    while True:
        start_time = time.time()
        current_state = my_drone.update_state()
        time_stamps.append(start_time)

        condition_for_avoiding_obstacle_is_true = condition_for_avoiding_obstacle(waypoint)
        
        if is_waypoint_reached(my_drone.position, waypoint, tolerance):
            break

        if condition_for_avoiding_obstacle_is_true and stop_time is None:
            stop_time = start_time

        control_input, next_state = mpc_control_drone(current_state, waypoint, my_drone.A, my_drone.B, Q, R, horizon, max_velocity, max_acceleration, goal, condition_for_avoiding_obstacle_is_true)
        my_drone.apply_control(control_input)

        x_positions.append(current_state[0])
        x_velocities.append(current_state[3])
        x_accelerations.append(control_input[0])

        print("waypoint         {:>6} {:>6} {:>6}".format(np.round(waypoint[0], 2), np.round(waypoint[1], 2), np.round(waypoint[2], 2)))

        if velocity_threshold > np.abs(current_state[3]) and acceleration_threshold > np.abs(control_input[0]):
            stop_drone = True
            break

        p.stepSimulation()

if len(time_stamps) > len(x_positions):
    time_stamps = time_stamps[:len(x_positions)]

# Compute relative time
time_x = np.array(time_stamps) - time_stamps[0]

# Plotting
# Plotting
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time_x, x_positions, label='Position of the quadrotor')
plt.title('X Position Over Time')
plt.xlabel('Time')
plt.ylabel('X Position')

plt.subplot(2, 1, 2)
plt.plot(time_x, x_velocities, label='Velocity of the quadrotor')
plt.title('X Velocity Over Time')
plt.xlabel('Time')
plt.ylabel('X Velocity')

plt.subplots_adjust(hspace=0.5)

if stop_time is not None:
    relative_stop_time = stop_time - time_stamps[0]
    plt.figure(1)  
    for i in range(1, 3):
        plt.subplot(2, 1, i)
        plt.axvline(x=(relative_stop_time-.25), color='red', linestyle=':', label='Obstacle detected, emergency stop initiated')    
        plt.legend(loc="upper left")

plt.show()