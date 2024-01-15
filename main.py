import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp

# Import the obstacles from the add_obstacles.py file and the move_the_column function
from control_scripts.add_obstacles import add_obstacles
from control_scripts.add_obstacles import move_the_column

# Import the drone class from the drone.py file
from drone import Drone

# Import the RRT*, IRRT classes and plot functions

from RRT_star.class_rrt_solovey import RRTStar_solovey as rrt_solovey
from RRT_star.class_rrt_informed import IRRT as rrt_informed
from RRT_star.class_rrt_solovey import plot_rrt
from RRT_star.class_rrt_solovey import plot_rrt_3d
from Metrics.plotting_trajectory_and_reference_combined import plot_trajectory_3d
from Metrics.plotting_trajectory_and_reference_combined import plot_trajectory_2d      

# Import the MPC control function, the waypoint reached function and the condition for avoiding obstacle function
from control_scripts.control_mpc import mpc_control_drone
from control_scripts.control_mpc import is_waypoint_reached
from control_scripts.control_mpc import condition_for_avoiding_obstacle

# Initialize the simulation, set the gravity and load the plane
p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

# Add obstacles to the simulation and list them in a list
obstacles, sliding_column_ids = add_obstacles()
obstacles = np.array(obstacles)

#==============================================================================
#========== BELOW VARIABLES CAN BE ADJUSTED FOR THE SIMULATION ================
#==============================================================================
#========== BELOW VARIABLES CAN BE ADJUSTED FOR THE SIMULATION ================
#==============================================================================


# Define the start, goal, step size, max iterations and gamma_kf for the RRT algorithms
#==============================================================================
start = np.array([0, 0, 0.25 + 0.5])
# goal = np.array([2.5, 4.5, 0.8])
goal = np.array([np.random.uniform(0.5, 2), np.random.uniform(4.5, 5), np.random.uniform(0.6, 0.9)])
step_size = 0.1
max_iter = 1000
gamma_kf = 1

#Uncomment the RRT* algorithm you want to use between informed and solovey
#==============================================================================

# rrt_inst = rrt_informed(start, goal, obstacles, step_size, max_iter, gamma_kf)
rrt_inst = rrt_solovey(start, goal, obstacles, step_size, max_iter, gamma_kf)

path, tree = rrt_inst.rrt_star_algorithm() # Run the RRT algorithm and get the path and tree

##Uncomment if you want to plot the 2D or 3D RRT algorithm
#==============================================================================

# plot_rrt_3d(tree, path, obstacles)
# plot_rrt(tree, path, obstacles)    


# MPC variables
#==============================================================================
horizon = 10
dt = 0.01
Q = np.diag([100, 100, 100, 0, 0, 0])
R = np.diag([0.001, 0.001, 0.001])
tolerance = 0.1 # Waypoint tolerance (when to switch to the next waypoint)
safety_margin = 1.3 # Safety margin for the obstacle avoidance, how far from the obstacle should the quadrotor start stopping

# Variables of the quadrotor instance
#==============================================================================
drone_model = "urdf_files/cf2x.urdf"
start_position = np.array([0, 0, 0.25]) 
drone_mass = 0.027 * 10**0
max_velocity = 8.333 #ms^-1
max_acceleration = 22.07 #ms^-2

#==============================================================================
#========== FINISHED ADJUSTING VARIABLES ======================================
#==============================================================================
#========== FINISHED ADJUSTING VARIABLES ======================================
#==============================================================================

### For plotting the 3D trajectory of the quadrotor ###
trajectory = []
previous_min_distance = float('inf') # Initialize the previous minimum distance to infinity

my_drone = Drone(drone_model, start_position, drone_mass, dt) # Initialize the drone instance


# Control (MPC) Loop
for waypoint in path:
    final_waypoint_reached = True if waypoint == path[-1] else False # Check if the final waypoint is reached
    if final_waypoint_reached:
        print("Final waypoint reached!")       
    while True:
        #### Start the timer for debug printing below ####
        # start_time = time.time()
        current_state = my_drone.update_state() # Update the state of the drone

        #### For plotting the 3D trajectory of the quadrotor ####
        trajectory.append(current_state[0:3])


        sliding_column_ids, velocity_columns = move_the_column(sliding_column_ids) # Move the sliding column and get column ids
        true_or_false, min_distance = condition_for_avoiding_obstacle_is_true = condition_for_avoiding_obstacle(my_drone.position, sliding_column_ids, safety_margin, previous_min_distance) # Check if the condition for avoiding the obstacle is true
        previous_min_distance = min_distance

        if not final_waypoint_reached:
            if is_waypoint_reached(my_drone.position, waypoint, tolerance):
                break  # Exit the loop and move to the next waypoint if waypoint is reached

        control_input = mpc_control_drone(current_state, waypoint,
                                                       my_drone.A, my_drone.B, Q, R, horizon, max_velocity,
                                                         max_acceleration, goal, true_or_false) # Calculate the control input
        my_drone.apply_control(control_input)

        #### Debug printing ####
        # print("waypoint         {:>6} {:>6} {:>6}".format(np.round(waypoint[0], 2), np.round(waypoint[1], 2), np.round(waypoint[2], 2)))
        # print("Difference       {:>6} {:>6} {:>6}".format(np.round(current_state[0]-waypoint[0], 2), np.round(current_state[1]-waypoint[1], 2), np.round(current_state[2]-waypoint[2], 2)))
        # print("Control Input:   {:>6} {:>6} {:>6}".format(np.round(control_input[0], 2), np.round(control_input[1], 2), np.round(control_input[2], 2)))
        # end_time = time.time()
        # control_frequency = 1.0 / (end_time - start_time)
        # print(f"Control frequency: {control_frequency} Hz")

        if final_waypoint_reached:
            velocity_stable = np.abs(current_state[3]).max() < 0.4
            acceleration_stable = np.abs(control_input[1]).max() < 0.2

            if velocity_stable and acceleration_stable:
                print("Drone is stable at final waypoint. Exiting loop.")
                time.sleep(5) # Wait for 5 seconds before exiting the simulation
                break
        

        p.stepSimulation()

# Plot the 2D and 3D trajectory of the quadrotor
#==============================================================================
        
# plot_trajectory_2d(trajectory, path, obstacles, tree) # Plot the 2D trajectory of the quadrotor
plot_trajectory_3d(trajectory, path, obstacles, tree) # Plot the 3D trajectory of the quadrotor
