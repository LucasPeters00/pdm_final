import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
import sys
import csv

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from RRT_star.class_rrt import plot_rrt
from RRT_star.class_rrt_solovey import RRTStar_solovey as rrt_scratch
from RRT_star.class_rrt_informed import IRRT as rrt_informed
from control_scripts.add_obstacles import add_obstacles

# Initialize the simulation
p.connect(p.DIRECT)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

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
    total_nodes = 0
    success_percentage = 0
    total_time = 0.0
    time_per_iteration = []
    nodes_per_iteration = []
    success_percentage_per_iteration = []
    path_lengths = []
    rrt_graphs = []
    nodes_per_ideal_path = []

    # Run the RRT* algorithm and show the results
    for i in range(num_iterations):
        start_time = time.time()  # Start de tijd voor deze iteratie

        # Dit moet handiger kunnen om sneller te runnen
        # Run RRT* algorithm
        if rrt_algorithm == rrt_scratch:
            rrt_inst = rrt_scratch(start, goal, obstacles, step_size, max_iter, gamma_kf)
        elif rrt_algorithm == rrt_informed:
            rrt_inst = rrt_informed(start, goal, obstacles, step_size, max_iter, gamma_kf)

        path, tree = rrt_inst.rrt_star_algorithm()

        # Check if a valid path is found
        if path is not None:
            total_nodes += len(tree)
            success_percentage += 1

            # Count the nodes along the ideal path
            nodes_along_ideal_path = len(path)
            nodes_per_ideal_path.append(nodes_along_ideal_path)

            # Collect data for analysis
            path_length = calculate_path_length(path)
            path_lengths.append(path_length)

        total_time += time.time() - start_time  # Calculate total time for all iterations
        time_per_iteration.append(time.time() - start_time)  # Calculate time for this iteration
        nodes_per_iteration.append(len(tree))  # Calculate nodes for this iteration
        success_percentage_per_iteration.append(1 if path is not None else 0)  # Calculate success percentage for this iteration

        # Reset simulation for next iteration
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])

    return total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths, rrt_graphs, nodes_per_ideal_path


# Function to calculate statistics
def statistics(nodes_per_ideal_path, num_iterations, total_nodes, success_percentage, total_time, time_per_iteration,
               nodes_per_iteration, success_percentage_per_iteration, path_lengths):
    # Compute statistics
    if len(path_lengths) > 0:
        mean_path_length = np.mean(path_lengths)
        std_path_length = np.std(path_lengths)
    else:
        mean_path_length = 0.0
        std_path_length = 0.0

    mean_nodes = total_nodes / num_iterations
    mean_success_percentage = (success_percentage / num_iterations) * 100
    mean_total_time = total_time / num_iterations
    std_nodes = np.std(nodes_per_iteration)
    std_success_percentage = np.std(success_percentage_per_iteration) * 100
    std_total_time = np.std(time_per_iteration)

    # Calculate mean nodes per ideal path
    mean_nodes_per_ideal_path = np.mean(nodes_per_ideal_path)
    return mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time, std_nodes, std_success_percentage, std_total_time, mean_nodes_per_ideal_path

# Add obstacles to the simulation and list them in a list
obstacles, _ = add_obstacles()
obstacles = np.array(obstacles)
# Define the start and goal positions and step size and max iterations
start = np.array([0, 0, 0.25 + 0.5])
# goal = np.array([np.random.uniform(-1, 3), np.random.uniform(4.5, 6), np.random.uniform(0.2, 1.2)])
# For RRT_star fixed goal is needed to compare the results
goal = np.array([2, 5.5, 0.2])

def save_results_to_csv(results, filename="results.csv"):
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['parameter_set', 'step_size', 'max_iter', 'num_iterations', 'rrt_algorithm', 'gamma_kf', 'mean_path_length', 'std_path_length', 'mean_nodes', 'mean_nodes_per_ideal_path', 'mean_success_percentage', 'mean_total_time', 'std_nodes', 'std_success_percentage', 'std_total_time']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for key, values in results.items():
            writer.writerow({
                'parameter_set': key,
                'step_size': values['step_size'],
                'max_iter': values['max_iter'],
                'num_iterations': values['num_iterations'],
                'rrt_algorithm': str(values['rrt_algorithm']),
                'gamma_kf': values['gamma_kf'],
                'mean_path_length': values['mean_path_length'],
                'std_path_length': values['std_path_length'],
                'mean_nodes': values['mean_nodes'],
                'mean_nodes_per_ideal_path': values['mean_nodes_per_ideal_path'],
                'mean_success_percentage': values['mean_success_percentage'],
                'mean_total_time': values['mean_total_time'],
                'std_nodes': values['std_nodes'],
                'std_success_percentage': values['std_success_percentage'],
                'std_total_time': values['std_total_time']
            })

# def variable_parameters(rrt_scratch, rrt_informed):
#     parameter_set = {
#         "parameter_set_1": {'step_size': 0.1, 'max_iter': 1000, 'num_iterations': 1, 'rrt_algorithm' : rrt_scratch, 'gamma_kf' : 1},
#         "parameter_set_2": {'step_size': 0.1, 'max_iter': 1000, 'num_iterations': 1, 'rrt_algorithm' : rrt_informed, 'gamma_kf' : 1}
#     }
#     return parameter_set

# def metrics():
#     parameter_set = variable_parameters(rrt_scratch, rrt_informed)
#     results = {}
#     for key, params in parameter_set.items():
#         step_size, max_iter, *_ = params.values()
#         num_iterations = params['num_iterations']

#         total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths, rrt_graphs, nodes_per_ideal_path = run_simulation_with_rrt_star(
#             start, goal, obstacles, step_size, max_iter, num_iterations, params['gamma_kf'], params['rrt_algorithm'])

#         mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time, std_nodes, std_success_percentage, std_total_time, mean_nodes_per_ideal_path = statistics(
#             nodes_per_ideal_path, num_iterations, total_nodes, success_percentage, total_time, time_per_iteration,
#             nodes_per_iteration, success_percentage_per_iteration, path_lengths)

#         results[key] = {
#             "step_size": step_size,
#             "max_iter": max_iter,
#             "num_iterations": num_iterations,
#             "rrt_algorithm": params['rrt_algorithm'],
#             "gamma_kf": params['gamma_kf'],
#             "mean_path_length": mean_path_length,
#             "std_path_length": std_path_length,
#             "mean_nodes": mean_nodes,
#             "mean_nodes_per_ideal_path": mean_nodes_per_ideal_path,
#             "mean_success_percentage": mean_success_percentage,
#             "mean_total_time": mean_total_time,
#             "std_nodes": std_nodes,
#             "std_success_percentage": std_success_percentage,
#             "std_total_time": std_total_time
#         }

#     save_results_to_csv(results)

#     return results

import itertools

# Function to perform grid search
# Function to perform grid search
def grid_search(rrt_algorithm_classes, max_iter_values, gamma_kf_values, step_size_values, num_iterations_values):
    results = {}

    for rrt_algorithm_class in rrt_algorithm_classes:
        parameter_combinations = itertools.product(max_iter_values, gamma_kf_values, step_size_values, num_iterations_values)

        for i, (max_iter, gamma_kf, step_size, num_iterations) in enumerate(parameter_combinations, start=1):
            key = f"parameter_set_{i}_{rrt_algorithm_class.__name__}"
            print(f"Running iteration {i} with parameters: max_iter={max_iter}, gamma_kf={gamma_kf}, step_size={step_size}, num_iterations={num_iterations}")

            total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths, rrt_graphs, nodes_per_ideal_path = run_simulation_with_rrt_star(
                start, goal, obstacles, step_size, max_iter, num_iterations, gamma_kf, rrt_algorithm_class)

            mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time, std_nodes, std_success_percentage, std_total_time, mean_nodes_per_ideal_path = statistics(
                nodes_per_ideal_path, num_iterations, total_nodes, success_percentage, total_time, time_per_iteration,
                nodes_per_iteration, success_percentage_per_iteration, path_lengths)

            results[key] = {
                "step_size": step_size,
                "max_iter": max_iter,
                "num_iterations": num_iterations,
                "rrt_algorithm": rrt_algorithm_class.__name__,
                "gamma_kf": gamma_kf,
                "mean_path_length": mean_path_length,
                "std_path_length": std_path_length,
                "mean_nodes": mean_nodes,
                "mean_nodes_per_ideal_path": mean_nodes_per_ideal_path,
                "mean_success_percentage": mean_success_percentage,
                "mean_total_time": mean_total_time,
                "std_nodes": std_nodes,
                "std_success_percentage": std_success_percentage,
                "std_total_time": std_total_time
            }

            save_results_to_csv(results)

    return results

# Define the ranges for grid search
max_iter_values = [1000, 5000, 10000]
gamma_kf_values = [1, 3, 5]
step_size_values = [0.1]
num_iterations_values = [50]
rrt_algorithm_classes = [rrt_scratch, rrt_informed]

# Perform grid search for each rrt_algorithm_class
grid_results = grid_search(rrt_algorithm_classes, max_iter_values, gamma_kf_values, step_size_values, num_iterations_values)







