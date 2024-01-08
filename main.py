import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
from control_scripts.add_obstacles import add_obstacles

# Import the Drone class from its script file
from drone import Drone

# Load the functions defined in other scripts
# from RRT_star.class_rrt import rrt_star
from RRT_star.class_rrt import plot_rrt
from RRT_star.class_rrt import plot_rrt_3d    
from control_scripts.add_obstacles import add_obstacles
from control_scripts.control_mpc import mpc_control_drone
from control_scripts.control_mpc import is_waypoint_reached
from control_scripts.add_obstacles import move_the_column
from control_scripts.control_mpc import condition_for_avoiding_obstacle

from RRT_star.class_rrt import RRTStar as rrt


# Initialize the simulation
p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

# Add obstacles to the simulation and list them in a list
obstacles, sliding_column_ids = add_obstacles()
obstacles = np.array(obstacles)

# Define the start and goal positions and step size and max iterations
start = np.array([0, 0, 0.25 + 0.5])
goal = np.array([np.random.uniform(-1, 3), np.random.uniform(4.5, 6), np.random.uniform(0.2, 1.2)])
step_size = 0.1
max_iter = 1000

# Run the RRT* algorithm and show the results
rrt_inst = rrt(start, goal, obstacles, step_size, max_iter)
path, tree = rrt_inst.rrt_star_algorithm()

### Plot the results comment or uncomment the line you want to see ###

# plot_rrt_3d(tree, path, obstacles)
plot_rrt(tree, path, obstacles)    

# Load the drone into the simulation
drone_model = "urdf_files/cf2x.urdf"
start_position = np.array([0, 0, 0.25]) 
drone_mass = 0.027 * 10**0

#Contraint for MPC 
max_velocity = 8.333 #ms^-1
max_acceleration = 22.07 #ms^-2

"""Total Mass (m): The mass of the drone, which is given as 0.027 kg in the <inertial> tag of the base_link element.
Thrust-to-Weight Ratio (T/W): Provided as 2.25. This ratio indicates how much thrust the drone can generate compared
 to its weight. The actual thrust (T) can be calculated by multiplying this ratio by the weight of the drone 
 (W = m * g, where g is the acceleration due to gravity, approximately 9.81 m/s² on Earth)."""

# MPC parameters
horizon = 10
dt = 0.01

# Define the MPC cost matrices, the velocity is not penalized
Q = np.diag([100, 100, 100, 0, 0, 0])
R = np.diag([0.001, 0.001, 0.001])

# Create a Drone instance
my_drone = Drone(drone_model, start_position, drone_mass, dt)

#Variables for MPC loop
tolerance = 0.1
safety_margin = 0.8


# Control (MPC) Loop
for waypoint in path:

    final_waypoint_reached = True if waypoint == path[-1] else False
    if final_waypoint_reached:
        print("Final waypoint reached!")        
    while True:
        
        # Start the timer
        start_time = time.time()
        current_state = my_drone.update_state()


        #Sliding columns 
        sliding_column_ids, velocity_columns = move_the_column(sliding_column_ids)


        condition_for_avoiding_obstacle_is_true = condition_for_avoiding_obstacle(my_drone.position, sliding_column_ids, safety_margin)

        # Check if the current waypoint is reached
        if not final_waypoint_reached:
            if is_waypoint_reached(my_drone.position, waypoint, tolerance):
                break  # Exit the loop and move to the next waypoint


        control_input, next_state = mpc_control_drone(current_state, waypoint,
                                                       my_drone.A, my_drone.B, Q, R, horizon, max_velocity,
                                                         max_acceleration, goal, condition_for_avoiding_obstacle_is_true)
        
        my_drone.apply_control(control_input)

        # Debug printing
        print("waypoint         {:>6} {:>6} {:>6}".format(np.round(waypoint[0], 2), np.round(waypoint[1], 2), np.round(waypoint[2], 2)))
        # print("Difference       {:>6} {:>6} {:>6}".format(np.round(current_state[0]-waypoint[0], 2), np.round(current_state[1]-waypoint[1], 2), np.round(current_state[2]-waypoint[2], 2)))
        print("Control Input:   {:>6} {:>6} {:>6}".format(np.round(control_input[0], 2), np.round(control_input[1], 2), np.round(control_input[2], 2)))

        # Stop the clock
        end_time = time.time()
        # Calculate the control frequency
        control_frequency = 1.0 / (end_time - start_time)

        print(f"Control frequency: {control_frequency} Hz")

        p.stepSimulation()
        # time.sleep(dt)
