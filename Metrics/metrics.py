import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import cvxpy as cp
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Now, import the module using the absolute path
from RRT_star.class_rrt import plot_rrt
from RRT_star.class_rrt import RRTStar as rrt

# Load the functions defined in other scripts
# from RRT_star.class_rrt import rrt_star
from control_scripts.add_obstacles import add_obstacles

# Initialize the simulation
p.connect(p.DIRECT)
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
        
# Create a dictionary that sets the variables for three different parameter sets. Variables are: step_size, max_iter, num_iterations
def variable_parameters():
    parameter_set = {
        "parameter_set_1": [0.3, 500, 5],
        # "parameter_set_2": [0.1, 10000, 1],
        # "parameter_set_3": [0.1, 50000, 1]
    }
    return parameter_set

# Add obstacles to the simulation and list them in a list
obstacles, _ = add_obstacles()
obstacles = np.array(obstacles)

# Define the start and goal positions and step size and max iterations
start = np.array([0, 0, 0.25 + 0.5])
# goal = np.array([np.random.uniform(-1, 3), np.random.uniform(4.5, 6), np.random.uniform(0.2, 1.2)])

# For RRT_star fixed goal is needed to compare the results
goal = np.array([2, 5.5, 0.2])
# step_size = 0.1     # Adjust as needed
# max_iter = 500      # Adjust as needed
# num_iterations = 10  # Adjust as needed

def calculate_path_length(path):
    total_length = 0.0
    for i in range(len(path) - 1):
        node1 = np.array(path[i])
        node2 = np.array(path[i + 1])
        distance = np.linalg.norm(node2 - node1)
        total_length += distance
    return total_length

# Run RRT* algorithm and simulation num_iterations times
def run_simulation_with_rrt_star(start, goal, obstacles, step_size, max_iter, num_iterations):
    # Initialize variablesa
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
    rrt_inst = rrt(start, goal, obstacles, step_size, max_iter)

    for i in range(num_iterations):
        start_time = time.time()  # Start de tijd voor deze iteratie

        # Run RRT* algorithm
        path, tree = rrt_inst.rrt_star_algorithm()

        # # Store RRT* graph data for visluasation later
        # rrt_graphs.append((path, tree))

        # Check if a valid path is found
        if path is not None:
            total_nodes += len(tree)  # Aangepast om de lengte van de boom te krijgen
            success_percentage += 1

            # Count the nodes along the ideal path
            nodes_along_ideal_path = len(path)
            nodes_per_ideal_path.append(nodes_along_ideal_path)

            # Run simulation with the generated path
            # run_simulation(drone_id, path)

            # Collect data for analysis
            path_length = calculate_path_length(path)
            path_lengths.append(path_length)

        total_time += time.time() - start_time  # Calculate total time for all iterations
        time_per_iteration.append(time.time() - start_time) # Calculate time for this iteration
        nodes_per_iteration.append(len(tree)) # Calculate nodes for this iteration
        success_percentage_per_iteration.append(1 if path is not None else 0) # Calculate success percentage for this iteration

        # Reset simulation for next iteration
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
        
    return total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths, rrt_graphs, nodes_per_ideal_path
   
def statistics(nodes_per_ideal_path, num_iterations, total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths): 
    # Compute statistics
    if len(path_lengths) > 0:
        mean_path_length = np.mean(path_lengths)
        std_path_length = np.std(path_lengths)
    else:
        mean_path_length = 0.0
        std_path_length = 0.0

    mean_nodes = total_nodes / num_iterations # Wat zegt dit --> wss niet relevant
    mean_success_percentage = (success_percentage / num_iterations) * 100
    mean_total_time = total_time / num_iterations
    std_nodes = np.std(nodes_per_iteration)
    std_success_percentage = np.std(success_percentage_per_iteration) * 100
    std_total_time = np.std(time_per_iteration)
     # Calculate mean nodes per ideal path
    mean_nodes_per_ideal_path = np.mean(nodes_per_ideal_path)
    return mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time, std_nodes, std_success_percentage, std_total_time, mean_nodes_per_ideal_path

# Create a function that runs the metrics function for different parameter sets
def metrics(parameter_set=variable_parameters()):
    results = {}

    for key, value in parameter_set.items():
        step_size, max_iter, num_iterations = value

        total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths, rrt_graphs, nodes_per_ideal_path = run_simulation_with_rrt_star(start, goal, obstacles, step_size, max_iter, num_iterations)
        mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time, std_nodes, std_success_percentage, std_total_time, mean_nodes_per_ideal_path = statistics(nodes_per_ideal_path, num_iterations, total_nodes, success_percentage, total_time, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, path_lengths)

        results[key] = {
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

    return results

# Run the metrics function
results = metrics()
print(results)

# Compare results for different parameter sets
for key, value in results.items():
    print(f"Results for {key}:")
    for metric, result in value.items():
        print(f"{metric}: {result}")
    print("\n")


def plot_statistics(num_iterations, path_lengths, time_per_iteration, nodes_per_iteration, success_percentage_per_iteration, mean_path_length, std_path_length, mean_nodes, mean_success_percentage, mean_total_time):
    # Print or use the computed statistics as needed
    print("Path Lengths", path_lengths)
    print("Mean Path Length:", mean_path_length)
    print("Standard Deviation of Path Length:", std_path_length)
    print("Mean Total Nodes:", mean_nodes)
    print("Mean Success Percentage:", mean_success_percentage)
    print("Mean Total Time:", mean_total_time)

    # Create a figure with subplots
    fig, axs = plt.subplots(4, 2, figsize=(15, 15))  # 4 rows, 2 columns

    # Plot path lengths
    axs[0, 0].plot(path_lengths)
    axs[0, 0].set_xlabel('Iteration')
    axs[0, 0].set_ylabel('Path Length')
    axs[0, 0].set_title('Path Lengths over provided Iterations')

    # Boxplot for mean path length and std deviation
    axs[0, 1].boxplot([path_lengths], labels=['Mean Path Length'])
    axs[0, 1].set_ylabel('Path Length')
    axs[0, 1].set_title('Boxplot of Mean Path Length over provided Iterations')

    # Plot time per iteration
    axs[1, 0].plot(time_per_iteration, marker='o')
    axs[1, 0].set_xlabel('Iteration')
    axs[1, 0].set_ylabel('Time (s)')
    axs[1, 0].set_title('Time per Iteration')

    # Plot nodes per iteration
    axs[1, 1].plot(nodes_per_iteration, marker='o')
    axs[1, 1].set_xlabel('Iteration')
    axs[1, 1].set_ylabel('Total Nodes')
    axs[1, 1].set_title('Total Nodes per Iteration')

    # Bar chart for success/failure per iteration
    axs[2, 0].bar(range(1, num_iterations + 1), success_percentage_per_iteration, color=['green' if success == 1 else 'red' for success in success_percentage_per_iteration])
    axs[2, 0].set_xlabel('Iteration')
    axs[2, 0].set_ylabel('Success/Failure')
    axs[2, 0].set_title('Success/Failure per Iteration')

    # Remove empty subplot
    fig.delaxes(axs[2, 1])

    # Boxplot for mean nodes and std deviation
    axs[3, 0].boxplot([nodes_per_iteration], labels=['Mean Nodes'])
    axs[3, 0].set_ylabel('Nodes')
    axs[3, 0].set_title('Boxplot of Mean Nodes over provided Iterations')

    # Boxplot for mean time and std deviation
    axs[3, 1].boxplot([time_per_iteration], labels=['Mean Time'])
    axs[3, 1].set_ylabel('Time (s)')
    axs[3, 1].set_title('Boxplot of Mean Time over provided Iterations')

    # Adjust layout for better spacing
    plt.tight_layout()

    # Show the combined plot
    plt.show()


def visualise_rrt_graphs(rrt_graphs, obstacles):
    num_iterations = len(rrt_graphs)

    # Create a figure with subplots
    fig, axs = plt.subplots(num_iterations, 2, figsize=(15, 5 * num_iterations))

    # Loop over each iteration and plot RRT* graph in a subplot
    for i, (path, tree) in enumerate(rrt_graphs):
        if obstacles is not None:
            plot_rrt(tree, path, obstacles, (axs[i, 0], axs[i, 1]))
        else:
            plot_rrt(tree, path, [], (axs[i, 0], axs[i, 1]))
        axs[i, 0].set_title(f'Iteration {i + 1}')
        axs[i, 0].set_xlabel('X')
        axs[i, 0].set_ylabel('Y')

    # Show the combined plot
    plt.show()


