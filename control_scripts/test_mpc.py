import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
import matplotlib.pyplot as plt 

# Import the Drone class from its script file
from drone import Drone

# Load the functions defined in other scripts
from test_enviroment.Diederik.RRT_star.rrt_star import rrt_star
from test_enviroment.Diederik.RRT_star.rrt_star import plot_rrt
from test_enviroment.Diederik.RRT_star.rrt_star import plot_rrt_3d    
from test_enviroment.Diederik.control_scripts.add_obstacles import add_obstacles
from test_enviroment.Diederik.control_scripts.control_mpc import mpc_control_drone
from test_enviroment.Diederik.control_scripts.control_mpc import is_waypoint_reached

# Initialize the simulation
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

# Add obstacles to the simulation and list them in a list
# obstacles = np.array(add_obstacles())

# Define the start and goal positions and step size and max iterations
start = np.array([0, 0, 0.7])
goal = np.array([2, 0, 0.7])
step_size = 0.1
max_iter = 1000

# Run the RRT* algorithm and show the results
# path, tree = rrt_star(start, goal, obstacles, step_size, max_iter)

path = np.array([start, goal])

### Plot the results comment or uncomment the line you want to see ###

# plot_rrt_3d(tree, path, obstacles)
# plot_rrt(tree, path, obstacles)    

# Load the drone into the simulation
drone_model = "cf2x.urdf"
start_position = [0, 0, 0.7]
drone_mass = 0.027 * 10**0
max_velocity = 8.333 #ms^-1
max_acceleration = 22.07 #ms^-2

"""Total Mass (m): The mass of the drone, which is given as 0.027 kg in the <inertial> tag of the base_link element.
Thrust-to-Weight Ratio (T/W): Provided as 2.25. This ratio indicates how much thrust the drone can generate compared
 to its weight. The actual thrust (T) can be calculated by multiplying this ratio by the weight of the drone 
 (W = m * g, where g is the acceleration due to gravity, approximately 9.81 m/s² on Earth)."""


# MPC parameters
horizon = 10
dt = 0.01

# Create a Drone instance
my_drone = Drone(drone_model, start_position, drone_mass, dt)


# Updated Cost function weights for 6D state and 3D control
Q = np.diag([10, 10, 1000, 0, 0, 0])
R = np.diag([0.01, 0.01, 0.001])

positions_x = []
velocities_x = []
accelerations_x = []

# MPC Loop
for waypoint in path:
    while True:
        current_state = my_drone.update_state()

        tolerance = 0.15

        # Check if the current waypoint is reached
        if is_waypoint_reached(my_drone.position, waypoint, tolerance):
            break  # Exit the loop and move to the next waypoint

        # Apply MPC control
        control_input, next_state = mpc_control_drone(current_state, waypoint, my_drone.A, my_drone.B, Q, R, horizon, max_velocity, max_acceleration)

        # Apply control input to the drone
        my_drone.apply_control(control_input)

        # Debug printing
        print("waypoint         {:>6} {:>6} {:>6}".format(np.round(waypoint[0], 2), np.round(waypoint[1], 2), np.round(waypoint[2], 2)))
        print("Difference       {:>6} {:>6} {:>6}".format(np.round(current_state[0]-waypoint[0], 2), np.round(current_state[1]-waypoint[1], 2), np.round(current_state[2]-waypoint[2], 2)))
        print("Control Input:   {:>6} {:>6} {:>6}".format(np.round(control_input[0], 2), np.round(control_input[1], 2), np.round(control_input[2], 2)))
        
        accelerations_x.append(control_input[0])
        velocities_x.append(current_state[3])
        positions_x.append(current_state[0])

        # Step the simulation
        p.stepSimulation()
        # time.sleep(dt)

# Plotting the results
time_steps = np.arange(len(positions_x))
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_steps, positions_x, marker='o')
plt.ylabel('Position X')
plt.title('Position, Velocity, and Acceleration on X-axis Over Time')

plt.subplot(3, 1, 2)
plt.plot(time_steps, velocities_x, marker='o')
plt.ylabel('Velocity X')

plt.subplot(3, 1, 3)
plt.plot(time_steps, accelerations_x, marker='o')
plt.ylabel('Acceleration X')
plt.xlabel('Time Step')

plt.tight_layout()
plt.show()