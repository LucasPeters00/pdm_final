import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
import sys
import csv
import itertools
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from RRT_star.class_rrt_solovey import RRTStar_solovey as rrt_solovey
from RRT_star.class_rrt_informed import IRRT as rrt_informed
from control_scripts.add_obstacles import add_obstacles_rrt_star_small_environment, add_obstacles_rrt_star_large_environment

#=====================================================================================================
#========== BELOW VARIABLES CAN BE ADJUSTED FOR THE COMPARISON BETWEEN RRT* AND IRRT* ================
#=====================================================================================================

# Define the start, goal, max_iter_values, step_size_values, num_iterations_values and gamma_kf_values for the RRT* algorithms
start = np.array([0, 0, 0.25 + 0.5]) # Start position for the RRT* algorithms
goal = np.array([5.5, 10.5, 0.8]) # Goal position for the RRT* algorithms
max_iter_values = [1000] # Maximum number of iterations to run the RRT* algorithms
gamma_kf_values = [1] # Gamma_kf value to use in the RRT* algorithms
step_size_values = [0.1] # Step size to take in the RRT* algorithms
num_iterations_values = [1] # Number of iterations to run the RRT* algorithms
rrt_algorithm_classes = [rrt_solovey, rrt_informed] # Uncomment if you only want to run one algorithm

# Adjust which environment to use to compare the RRT* algorithms
# obstacles = add_obstacles_rrt_star_small_environment() # use this for the small environment comparison (also used for the MPC comparison)
obstacles = add_obstacles_rrt_star_large_environment() # use this for the large environment comparison
obstacles = np.array(obstacles)
#===================================================================================================== 

# Function to calculate path length
def calculate_path_length(path):
    total_length = 0.0
    for i in range(len(path) - 1):
        node1 = np.array(path[i])
        node2 = np.array(path[i + 1])
        distance = np.linalg.norm(node2 - node1)
        total_length += distance
    return total_length

# Function to run the RRT* algorithm and simulation
def run_simulation_with_rrt_star(start, goal, obstacles, step_size, max_iter, num_iterations, gamma_kf, rrt_algorithm):
    # Initialize variables
    time_per_iteration = []
    success_percentage_per_iteration = []
    path_lengths = []
    nodes_per_ideal_path = []

    # Run the RRT* algorithm and show the results
    for i in range(num_iterations):
        start_time = time.time()  

        if rrt_algorithm == rrt_solovey:
            rrt_inst = rrt_solovey(start, goal, obstacles, step_size, max_iter, gamma_kf)
        elif rrt_algorithm == rrt_informed:
            rrt_inst = rrt_informed(start, goal, obstacles, step_size, max_iter, gamma_kf)
 
        path, _ = rrt_inst.rrt_star_algorithm()

        # Check if a valid path is found
        if path is not None:
            # Count the nodes along the ideal path
            nodes_per_ideal_path.append(len(path))

            # Collect data for analysis
            path_length = calculate_path_length(path)
            path_lengths.append(path_length)

        time_per_iteration.append(time.time() - start_time) 
        success_percentage_per_iteration.append(1 if path is not None else 0)  

    return time_per_iteration, success_percentage_per_iteration, path_lengths, nodes_per_ideal_path

# Function to calculate statistics
def statistics(nodes_per_ideal_path, time_per_iteration, success_percentage_per_iteration, path_lengths):
    # Compute statistics
    if len(path_lengths) > 0:
        mean_path_length = np.mean(path_lengths)
        std_path_length = np.std(path_lengths)
    else:
        mean_path_length = 0.0
        std_path_length = 0.0

    mean_nodes_per_ideal_path = np.mean(nodes_per_ideal_path)
    std_nodes_per_ideal_path = np.std(nodes_per_ideal_path)
    mean_success_percentage = np.mean(success_percentage_per_iteration) * 100
    mean_time_per_iteration = np.mean(time_per_iteration)
    std_success_percentage = np.std(success_percentage_per_iteration) * 100
    std_time_per_iteration = np.std(time_per_iteration)
   
    return mean_path_length, std_path_length, mean_success_percentage, mean_time_per_iteration, std_success_percentage, std_time_per_iteration, mean_nodes_per_ideal_path, std_nodes_per_ideal_path

def save_results_to_csv(results, filename="results.csv"):
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['parameter_set', 'step_size', 'max_iter', 'num_iterations', 'rrt_algorithm', 'gamma_kf',
                      'mean_path_length', 'std_path_length', 'mean_success_percentage', 'std_success_percentage',
                      'mean_time_per_iteration', 'std_time_per_iteration', 'mean_nodes_per_ideal_path',
                      'std_nodes_per_ideal_path']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for key, values in results.items():
            writer.writerow({
                'parameter_set': values['parameter_set'],
                'step_size': values['step_size'],
                'max_iter': values['max_iter'],
                'num_iterations': values['num_iterations'],
                'rrt_algorithm': str(values['rrt_algorithm']),
                'gamma_kf': values['gamma_kf'],
                'mean_path_length': values['mean_path_length'],
                'std_path_length': values['std_path_length'],
                'mean_success_percentage': values['mean_success_percentage'],
                'std_success_percentage': values['std_success_percentage'],
                'mean_time_per_iteration': values['mean_time_per_iteration'],
                'std_time_per_iteration': values['std_time_per_iteration'],
                'mean_nodes_per_ideal_path': values['mean_nodes_per_ideal_path'],
                'std_nodes_per_ideal_path': values['std_nodes_per_ideal_path']
            })


# Function to perform grid search
def grid_search(rrt_algorithm_classes, max_iter_values, gamma_kf_values, step_size_values, num_iterations_values):
    results = {}
    counter = 0  # Counter to keep track of parameter sets

    for rrt_algorithm_class in rrt_algorithm_classes:
        parameter_combinations = itertools.product(max_iter_values, gamma_kf_values, step_size_values, num_iterations_values)

        for max_iter, gamma_kf, step_size, num_iterations in parameter_combinations:
            counter += 1  # Increment counter for each parameter set
            key = f"parameter_set_{counter}_{rrt_algorithm_class.__name__}"
            print(f"Running iteration {counter} with parameters: max_iter={max_iter}, gamma_kf={gamma_kf}, step_size={step_size}, num_iterations={num_iterations}")

            time_per_iteration, success_percentage_per_iteration, path_lengths, nodes_per_ideal_path = run_simulation_with_rrt_star(start, goal, obstacles, step_size, max_iter, num_iterations, gamma_kf, rrt_algorithm_class)

            mean_path_length, std_path_length, mean_success_percentage, mean_time_per_iteration, std_success_percentage, std_time_per_iteration, mean_nodes_per_ideal_path, std_nodes_per_ideal_path = statistics(nodes_per_ideal_path, time_per_iteration, success_percentage_per_iteration, path_lengths)

            results[key] = {
                'parameter_set': counter,
                'step_size': step_size,
                'max_iter': max_iter,
                'num_iterations': num_iterations,
                'rrt_algorithm': str(rrt_algorithm_class),
                'gamma_kf': gamma_kf,
                'mean_path_length': mean_path_length,
                'std_path_length': std_path_length,
                'mean_success_percentage': mean_success_percentage,
                'std_success_percentage': std_success_percentage,
                'mean_time_per_iteration': mean_time_per_iteration,
                'std_time_per_iteration': std_time_per_iteration,
                'mean_nodes_per_ideal_path': mean_nodes_per_ideal_path,
                'std_nodes_per_ideal_path': std_nodes_per_ideal_path
            }

            save_results_to_csv(results)

    return results

# Perform grid search for each rrt_algorithm_class
grid_results = grid_search(rrt_algorithm_classes, max_iter_values, gamma_kf_values, step_size_values, num_iterations_values)




