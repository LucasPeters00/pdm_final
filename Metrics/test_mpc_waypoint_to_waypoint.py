import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
# Import the Drone class from its script file
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from drone import Drone

import matplotlib.pyplot as plt

# Initialize the simulation
p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])



# Define the start and goal positions 
start = np.array([0, 0, 0.25 + 0.5])
goal = np.array([0, 1, 0.25 + 0.5])

path = np.array([start, goal])
# Load the drone into the simulation
drone_model = "urdf_files/cf2x.urdf"
start_position = np.array([0, 0, 0.25 + 0.5]) 
drone_mass = 0.027 * 10**0

#Contraint for MPC 
max_velocity = 8.333 #ms^-1
max_acceleration = 22.07 #ms^-2

# MPC parameters
horizon = 30
dt = 0.01

# Define the MPC cost matrices, the velocity is not penalized
Q = np.diag([100, 100, 100, 0, 0, 0])
R = np.diag([0.001, 0.001, 0.001])

# Create a Drone instance
my_drone = Drone(drone_model, start_position, drone_mass, dt)

#Variables for MPC loop
tolerance = 0.1
safety_margin = 0.7

def mpc_control_drone(x_init, waypoint, A, B, Q, R, horizon, max_velocity,
                       max_acceleration, goal, condition_for_avoiding_obstacle_is_true):
    # Pad the waypoint with zeros to match 6D state
    waypoint_padded = np.concatenate((waypoint, np.zeros(3)))

    if waypoint[0] == goal[0] and waypoint[1] == goal[1] and waypoint[2] == goal[2]:
        Q = np.diag([100, 100, 100, 0.1, 0.1, 0.1])
        
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

# Control (MPC) Loop

all_states = []
all_input = []
comp_times = []

velocity_threshold = 0.4  
acceleration_threshold = 0.2  

start_time = time.time()
for waypoint in path:

    final_waypoint_reached = True if np.array_equal(waypoint, path[-1]) else False
    # if final_waypoint_reached:
    #     print("Final waypoint reached!")        
    while True:
        
        # Start the timer
        # start_time = time.time()
        current_state = my_drone.update_state()
        all_states.append(current_state)

        # Check if the current waypoint is reached
        if not final_waypoint_reached:
            if is_waypoint_reached(my_drone.position, waypoint, tolerance):
                break  # Exit the loop and move to the next waypoint
        

        control_input, next_state = mpc_control_drone(current_state, waypoint,
                                                       my_drone.A, my_drone.B, Q, R, horizon, max_velocity,
                                                         max_acceleration, goal, condition_for_avoiding_obstacle_is_true=False)
        
        my_drone.apply_control(control_input)

        
        all_input.append(control_input)

        # print("current_state: ", current_state)

        if final_waypoint_reached:
            velocity_stable = np.abs(current_state[4]).max() < velocity_threshold
            acceleration_stable = np.abs(control_input[1]).max() < acceleration_threshold

            if velocity_stable and acceleration_stable:
                print("Drone is stable at final waypoint. Exiting loop.")
                break

        # # Debug printing
        # print("waypoint         {:>6} {:>6} {:>6}".format(np.round(waypoint[0], 2), np.round(waypoint[1], 2), np.round(waypoint[2], 2)))
        # # print("Difference       {:>6} {:>6} {:>6}".format(np.round(current_state[0]-waypoint[0], 2), np.round(current_state[1]-waypoint[1], 2), np.round(current_state[2]-waypoint[2], 2)))
        # print("Control Input:   {:>6} {:>6} {:>6}".format(np.round(control_input[0], 2), np.round(control_input[1], 2), np.round(control_input[2], 2)))

        # # Stop the clock
        # end_time = time.time()
        # # Calculate the control frequency
        # control_frequency = 1.0 / (end_time - start_time)

        # print(f"Control frequency: {control_frequency} Hz")

        p.stepSimulation()
        # time.sleep(dt)

end_time = time.time()
comp_time = end_time - start_time
comp_times.append(comp_time)

# Convert all_states to a numpy array
all_states = np.array(all_states)
all_input = np.array(all_input)
# Time vector for plotting
t = np.linspace(0, len(all_states) * dt, len(all_states))

# Plotting Position, Velocity, and Acceleration
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t, all_states[:, 1])
plt.title('X-axis Position Over Time')
plt.ylabel('Position (m)')

plt.subplot(3, 1, 2)
plt.plot(t, all_states[:, 4])
plt.title('X-axis Velocity Over Time')
plt.ylabel('Velocity (m/s)')

t = np.linspace(0, len(all_input) * dt, len(all_input))
plt.subplot(3, 1, 3)
plt.plot(t, all_input[:, 1])
plt.title('X-axis Acceleration Over Time')
plt.ylabel('Acceleration (m/s²)')
plt.xlabel('Time (s)')

plt.show()

np.save("all_states_h30.npy", all_states)
np.save("all_input_h30.npy", all_input)
np.save("comp_times_h30.npy", comp_time)
print("saved")