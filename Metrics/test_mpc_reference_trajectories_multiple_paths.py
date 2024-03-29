import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from drone import Drone  # Import the Drone class from its script file
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Initialize the simulation
p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

# Define the start and goal positions 
start = np.array([0, 0, 0.25 + 0.5])
goal = np.array([0, 1, 0.25 + 0.5])


# # path of sinus wave
path_sinus_wave = [
    [0.0, 0.0, 0.75], [0.06, 0.06, 0.75], [0.13, 0.13, 0.75], [0.19, 0.19, 0.75],
    [0.25, 0.25, 0.75], [0.32, 0.31, 0.75], [0.38, 0.37, 0.75], [0.44, 0.43, 0.75],
    [0.51, 0.49, 0.75], [0.57, 0.54, 0.75], [0.63, 0.59, 0.75], [0.70, 0.64, 0.75],
    [0.76, 0.69, 0.75], [0.83, 0.73, 0.75], [0.89, 0.78, 0.75], [0.95, 0.81, 0.75],
    [1.02, 0.85, 0.75], [1.08, 0.88, 0.75], [1.14, 0.91, 0.75], [1.21, 0.93, 0.75],
    [1.27, 0.95, 0.75], [1.33, 0.97, 0.75], [1.40, 0.98, 0.75], [1.46, 0.99, 0.75],
    [1.52, 1.00, 0.75], [1.59, 1.00, 0.75], [1.65, 1.00, 0.75], [1.71, 0.99, 0.75],
    [1.78, 0.98, 0.75], [1.84, 0.96, 0.75], [1.91, 0.94, 0.75], [1.97, 0.92, 0.75],
    [2.03, 0.90, 0.75], [2.10, 0.87, 0.75], [2.16, 0.83, 0.75], [2.22, 0.80, 0.75],
    [2.29, 0.76, 0.75], [2.35, 0.71, 0.75], [2.41, 0.67, 0.75], [2.48, 0.62, 0.75],
    [2.54, 0.57, 0.75], [2.61, 0.51, 0.75], [2.67, 0.46, 0.75], [2.73, 0.40, 0.75],
    [2.80, 0.34, 0.75], [2.86, 0.28, 0.75], [2.92, 0.22, 0.75], [2.99, 0.16, 0.75],
    [3.05, 0.09, 0.75], [3.11, 0.03, 0.75], [3.18, -0.03, 0.75], [3.24, -0.09, 0.75],
    [3.30, -0.16, 0.75], [3.37, -0.22, 0.75], [3.43, -0.28, 0.75], [3.50, -0.34, 0.75],
    [3.56, -0.40, 0.75], [3.62, -0.46, 0.75], [3.69, -0.51, 0.75], [3.75, -0.57, 0.75],
    [3.81, -0.62, 0.75], [3.88, -0.67, 0.75], [3.94, -0.71, 0.75], [4.00, -0.76, 0.75],
    [4.07, -0.80, 0.75], [4.13, -0.83, 0.75], [4.19, -0.87, 0.75], [4.26, -0.90, 0.75],
    [4.32, -0.92, 0.75], [4.39, -0.94, 0.75], [4.45, -0.96, 0.75], [4.51, -0.98, 0.75],
    [4.58, -0.99, 0.75], [4.64, -1.00, 0.75], [4.70, -1.00, 0.75], [4.77, -1.00, 0.75],
    [4.83, -0.99, 0.75], [4.89, -0.98, 0.75], [4.96, -0.97, 0.75], [5.02, -0.95, 0.75],
    [5.08, -0.93, 0.75], [5.15, -0.91, 0.75], [5.21, -0.88, 0.75], [5.28, -0.85, 0.75],
    [5.34, -0.81, 0.75], [5.40, -0.78, 0.75], [5.47, -0.73, 0.75], [5.53, -0.69, 0.75],
    [5.59, -0.64, 0.75], [5.66, -0.59, 0.75], [5.72, -0.54, 0.75], [5.78, -0.49, 0.75],
    [5.85, -0.43, 0.75], [5.91, -0.37, 0.75], [5.97, -0.31, 0.75], [6.04, -0.25, 0.75],
    [6.10, -0.19, 0.75], [6.17, -0.13, 0.75], [6.23, -0.06, 0.75], [6.29, 0.0, 0.75]
]

# # #path pyramid
path_pyramid = [
    [0.0, 0.0, 0.75], [0.17, 0.11, 0.75], [0.35, 0.22, 0.75], [0.52, 0.33, 0.75],
    [0.70, 0.44, 0.75], [0.87, 0.56, 0.75], [1.05, 0.67, 0.75], [1.22, 0.78, 0.75],
    [1.40, 0.89, 0.75], [1.57, 1.0, 0.75], [1.57, 1.0, 0.75], [1.75, 0.89, 0.75],
    [1.92, 0.78, 0.75], [2.09, 0.67, 0.75], [2.27, 0.56, 0.75], [2.44, 0.44, 0.75],
    [2.62, 0.33, 0.75], [2.79, 0.22, 0.75], [2.97, 0.11, 0.75], [3.14, 0.0, 0.75],
    [3.14, 0.0, 0.75], [3.32, -0.11, 0.75], [3.49, -0.22, 0.75], [3.67, -0.33, 0.75],
    [3.84, -0.44, 0.75], [4.01, -0.56, 0.75], [4.19, -0.67, 0.75], [4.36, -0.78, 0.75],
    [4.54, -0.89, 0.75], [4.71, -1.0, 0.75], [4.71, -1.0, 0.75], [4.89, -0.89, 0.75],
    [5.06, -0.78, 0.75], [5.24, -0.67, 0.75], [5.41, -0.56, 0.75], [5.59, -0.44, 0.75],
    [5.76, -0.33, 0.75], [5.93, -0.22, 0.75], [6.11, -0.11, 0.75], [6.28, 0.0, 0.75]
]

#path steps
path_steps = [
    [0.0, 0.0, 0.75], [0.0, 0.25, 0.75], [0.0, 0.5, 0.75], [0.0, 0.75, 0.75], [0.0, 1.0, 0.75],
    [0.0, 1.0, 0.75], [0.39, 1.0, 0.75], [0.79, 1.0, 0.75], [1.18, 1.0, 0.75], [1.57, 1.0, 0.75],
    [1.57, 1.0, 0.75], [1.57, 0.5, 0.75], [1.57, 0.0, 0.75], [1.57, -0.5, 0.75], [1.57, -1.0, 0.75],
    [1.57, -1.0, 0.75], [1.96, -1.0, 0.75], [2.36, -1.0, 0.75], [2.75, -1.0, 0.75], [3.14, -1.0, 0.75],
    [3.14, -1.0, 0.75], [3.14, -0.5, 0.75], [3.14, 0.0, 0.75], [3.14, 0.5, 0.75], [3.14, 1.0, 0.75],
    [3.14, 1.0, 0.75], [3.53, 1.0, 0.75], [3.93, 1.0, 0.75], [4.32, 1.0, 0.75], [4.71, 1.0, 0.75],
    [4.71, 1.0, 0.75], [4.71, 0.5, 0.75], [4.71, 0.0, 0.75], [4.71, -0.5, 0.75], [4.71, -1.0, 0.75],
    [4.71, -1.0, 0.75], [5.11, -1.0, 0.75], [5.50, -1.0, 0.75], [5.89, -1.0, 0.75], [6.28, -1.0, 0.75],
    [6.28, -1.0, 0.75], [6.28, -0.75, 0.75], [6.28, -0.5, 0.75], [6.28, -0.25, 0.75], [6.28, 0.0, 0.75]
]

# #unsemetrically wave
path_unsymmetrical_wave = [(0.0000, 0.0000, 0.75), (0.0635, 0.1209, 0.75),(0.1269, 0.2344, 0.75),(0.1904, 0.3338, 0.75),
(0.2539, 0.4139, 0.75),(0.3173, 0.4712, 0.75),(0.3808, 0.5044, 0.75),(0.4443, 0.5147, 0.75),(0.5077, 0.5053, 0.75),(0.5712, 0.4813, 0.75),(0.6347, 0.4491, 0.75),
(0.6981, 0.4159, 0.75),(0.7616, 0.3885, 0.75),(0.8251, 0.3732, 0.75),(0.8885, 0.3749, 0.75),(0.9520, 0.3964, 0.75),
(1.0155, 0.4384, 0.75),(1.0789, 0.4993, 0.75),(1.1424, 0.5754, 0.75),(1.2059, 0.6612, 0.75),(1.2693, 0.7498, 0.75),
(1.3328, 0.8340, 0.75),(1.3963, 0.9066, 0.75),(1.4597, 0.9614, 0.75),(1.5232, 0.9934, 0.75),(1.5867, 1.0000, 0.75),(1.6501, 0.9805, 0.75),
(1.7136, 0.9366, 0.75),(1.7771, 0.8722, 0.75),(1.8405, 0.7929, 0.75),(1.9040, 0.7056, 0.75),(1.9675, 0.6175, 0.75),(2.0309, 0.5358, 0.75),(2.0944, 0.4667, 0.75),(2.1579, 0.4149, 0.75),(2.2213, 0.3831, 0.75),(2.2848, 0.3717, 0.75),(2.3483, 0.3790, 0.75),(2.4117, 0.4010, 0.75),(2.4752, 0.4322, 0.75),(2.5387, 0.4658, 0.75),
(2.6021, 0.4947, 0.75),(2.6656, 0.5122, 0.75),(2.7291, 0.5122, 0.75),(2.7925, 0.4908, 0.75),(2.8560, 0.4455, 0.75),(2.9195, 0.3765, 0.75),(2.9829, 0.2862, 0.75),(3.0464, 0.1790, 0.75),(3.1099, 0.0609, 0.75),(3.1733, -0.0609, 0.75),(3.2368, -0.1790, 0.75),(3.3003, -0.2862, 0.75),(3.3637, -0.3765, 0.75),(3.4272, -0.4455, 0.75),(3.4907, -0.4908, 0.75),(3.5541, -0.5122, 0.75),(3.6176, -0.5122, 0.75),(3.6811, -0.4947, 0.75),(3.7445, -0.4658, 0.75),(3.8080, -0.4322, 0.75),(3.8715, -0.4010, 0.75),(3.9349, -0.3790, 0.75),(3.9984, -0.3717, 0.75),(4.0619, -0.3831, 0.75),
(4.1253, -0.4149, 0.75),(4.1888, -0.4667, 0.75),(4.2523, -0.5358, 0.75),(4.3157, -0.6175, 0.75),(4.3792, -0.7056, 0.75),(4.4427, -0.7929, 0.75),(4.5061, -0.8722, 0.75),(4.5696, -0.9366, 0.75),(4.6331, -0.9805, 0.75),(4.6965, -1.0000, 0.75),(4.7600, -0.9934, 0.75),(4.8235, -0.9614, 0.75),(4.8869, -0.9066, 0.75),(4.9504, -0.8340, 0.75),(5.0139, -0.7498, 0.75),(5.0773, -0.6612, 0.75),(5.1408, -0.5754, 0.75),(5.2043, -0.4993, 0.75),(5.2677, -0.4384, 0.75),(5.3312, -0.3964, 0.75),(5.3947, -0.3749, 0.75),(5.4581, -0.3732, 0.75),(5.5216, -0.3885, 0.75),(5.5851, -0.4159, 0.75),(5.6485, -0.4491, 0.75),(5.7120, -0.4813, 0.75),(5.7755, -0.5053, 0.75),(5.8389, -0.5147, 0.75),(5.9024, -0.5044, 0.75),(5.9659, -0.4712, 0.75),(6.0293, -0.4139, 0.75),(6.0928, -0.3338, 0.75),(6.1563, -0.2344, 0.75),
(6.2197, -0.1209, 0.75),
(6.2832, -0.0000, 0.75)]

# paths = [path_sinus_wave, path_pyramid, path_steps, path_unsymmetrical_wave]
paths = [path_steps, path_unsymmetrical_wave, path_pyramid, path_sinus_wave]

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

# Function definitions (mpc_control_drone, is_waypoint_reached)

def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity,
                       max_acceleration, goal, condition_for_avoiding_obstacle_is_true=False):
    # Pad the waypoint with zeros to match 6D state
    waypoint_padded = np.concatenate((waypoint, np.zeros(3)))

    if waypoint[0] == goal[0] and waypoint[1] == goal[1] and waypoint[2] == goal[2]:
        Q = np.diag([100, 100, 100, 0.1, 0.1, 0.1])
        
    # Initialize state (6D) and control (3D) variables
    x = cp.Variable((6, horizon + 1))
    u = cp.Variable((3, horizon))
    safety_margin = 0.7
    cost = 0
    # Initial state constraint, make sure the drone starts at the initial position
    constraints = [x[:, 0] == x_init]

    # MPC problem formulation
    for t in range(horizon):

        cost += cp.quad_form(x[:, t] - waypoint_padded, Q)

        if t < horizon - 1:
            cost += cp.quad_form(u[:, t], R)
            constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t]]
            constraints += [cp.abs(x[3:6, t]) <= max_velocity]  # Velocity constraints
            constraints += [cp.abs(u[:, t]) <= max_acceleration] # Acceleration constraints

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

# Plotting
# plt.figure(figsize=(12, 10))
fig, axs = plt.subplots(len(paths), 1, figsize=(10, len(paths)*3)) 

for i, path in enumerate(paths):
    # Reset the simulation for each path
    p.resetSimulation()
    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
    my_drone = Drone(drone_model, start_position, drone_mass, dt)
    
    all_states = []
    all_input = []

    for waypoint in path:
        final_waypoint_reached = True if np.array_equal(waypoint, path[-1]) else False
        while True:
            current_state = my_drone.update_state()
            all_states.append(current_state)
            if not final_waypoint_reached and is_waypoint_reached(my_drone.position, waypoint, tolerance):
                break
            control_input, next_state = mpc_control_drone(current_state, waypoint,
                                                       my_drone.A, my_drone.B, Q, R, horizon, max_velocity,
                                                         max_acceleration, goal, condition_for_avoiding_obstacle_is_true=False)
            my_drone.apply_control(control_input)
            all_input.append(control_input)
            if final_waypoint_reached: 
                print("Simulation:"+str(i+1)+" finished")
                break
            p.stepSimulation()

    all_states = np.array(all_states)
    path_array = np.transpose(np.array(path))

    axs[i].plot(all_states[:, 0], all_states[:, 1], label='Actual Trajectory')
    axs[i].plot(path_array[0], path_array[1], 'r--', label='Reference Trajectory')
    axs[i].set_title(f'Path {i+1}: Quadrotor Trajectory vs Reference Trajectory')
    axs[i].set_xlabel('X Position (m)')
    axs[i].set_ylabel('Y Position (m)')
    axs[i].set_xlim([0, 6.5])  # Set x-axis limits
    axs[i].set_ylim([-1.5, 1.5])  # Set y-axis limits

    axs[i].legend(loc='upper right')
    
plt.subplots_adjust(hspace = 0.5) 
plt.show()
