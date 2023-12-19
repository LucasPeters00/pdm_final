import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rrt_star(start, goal, obstacles, step_size, max_iter):
    # Initialize the tree with the start node
    print(start)
    print(goal) 
    print(obstacles)
    print(step_size)
    print(max_iter)
    
    tree = {tuple(start): {'parent': None, 'cost': 0}}

    best_path = None
    best_cost = np.inf

    for i in range(max_iter):

        if i % (max_iter // 50) == 0:
            print(f"Progress: {i / max_iter * 100}%")

        # Find the bounds of the configuration space based on the obstacles & Generate a random point within the bounds
        goal_sampling_rate = 0.05  # Adjust this value as needed

        if np.random.random() < goal_sampling_rate:
            random_point = goal
        else:
            x_min, y_min, z_min = 0, 0, 0
            x_max, y_max, z_max = 4, 6, 1.5
            random_point = np.array([np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max), np.random.uniform(z_min, z_max)])

        # Find the nearest node in the tree
        distances = []
        for node in tree.keys():
            node_array = np.array(node)
            distance = np.linalg.norm(node_array - random_point)
            distances.append(distance)
        nearest_node_index = np.argmin(distances)
        nearest_node = list(tree.keys())[nearest_node_index]

        # Create a new node
        direction_vector = random_point - nearest_node
        normalized_direction = direction_vector / np.linalg.norm(direction_vector)
        new_node_position = nearest_node + step_size * normalized_direction
        new_node = tuple(new_node_position)

        # Check for collision
        collision = False
        for column in obstacles:
            new_node_array = np.array(new_node[:2])
            column_center = np.array(column[:2])
            distance = np.linalg.norm(new_node_array - column_center)
            if distance < column[3]:
                collision = True
                break
        if collision:
            continue

        # Find the nearest neighbors within a radius
        neighbors = []
        for node in tree.keys():
            node_array = np.array(node)
            distance = np.linalg.norm(node_array - np.array(new_node))
            if distance < obstacles[0][3]:
                neighbors.append(node)

        # If there are no neighbors, continue with the loop
        if not neighbors:
            continue

        # Find the neighbor with the lowest cost
        costs = []
        for neighbor in neighbors:
            neighbor_cost = tree[neighbor]['cost']
            distance_to_new_node = np.linalg.norm(np.array(neighbor) - np.array(new_node))
            total_cost = neighbor_cost + distance_to_new_node
            costs.append(total_cost)
        min_cost_index = np.argmin(costs)
        min_cost_neighbor = neighbors[min_cost_index]

        # Add the new node to the tree with the neighbor with the lowest cost as the parent
        parent_cost = tree[min_cost_neighbor]['cost']
        distance_to_parent = np.linalg.norm(np.array(min_cost_neighbor) - np.array(new_node))
        new_node_cost = parent_cost + distance_to_parent
        tree[new_node] = {'parent': min_cost_neighbor, 'cost': new_node_cost}

        # Rewire the tree
        for neighbor in neighbors:
            distance_to_neighbor = np.linalg.norm(np.array(new_node) - np.array(neighbor))
            potential_cost = new_node_cost + distance_to_neighbor
            if potential_cost < tree[neighbor]['cost']:
                tree[neighbor] = {'parent': new_node, 'cost': potential_cost}

        # Check if we've reached the goal
        distance_to_goal = np.linalg.norm(np.array(new_node) - np.array(goal))
        if distance_to_goal < step_size:
            goal_cost = new_node_cost + distance_to_goal
            if goal_cost < best_cost:
                best_cost = goal_cost
                tree[tuple(goal)] = {'parent': new_node, 'cost': goal_cost}
                print("Path found or better path found")

                # Construct the path
                path = []
                current_node = tuple(goal)
                while current_node is not None:
                    path.append(current_node)
                    current_node = tree[current_node]['parent']
                path.reverse()  # Reverse the path to go from start to goal

                best_path = path
        
    # Return the best path found
    return best_path, tree


def plot_rrt(tree, path, obstacles):
    fig, axs = plt.subplots(1, 2)  # Create 2 subplots side by side

    # Plot for X-Y plane
    ax = axs[0]
    ax.set_title('X-Y Plane')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    for obstacle in obstacles:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[3], color='red')
        ax.add_patch(circle)

    if tree is not None:
        for node in tree.keys():
            ax.plot(node[0], node[1], marker='o', color='black', markersize=.2)

        for node, data in tree.items():
            if data['parent'] is not None:
                ax.plot([node[0], data['parent'][0]], [node[1], data['parent'][1]], color='green', linewidth=0.4)

    if path is not None:
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color='blue', linewidth=2)

    # Plot for X-Z plane
    ax = axs[1]
    ax.set_title('X-Z Plane')
    ax.set_xlabel('X')
    ax.set_ylabel('Z')

    if tree is not None:
        for node in tree.keys():
            ax.plot(node[0], node[2], marker='o', color='black', markersize=.2)

        for node, data in tree.items():
            if data['parent'] is not None:
                ax.plot([node[0], data['parent'][0]], [node[2], data['parent'][2]], color='green', linewidth=0.4)

    if path is not None:
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][2], path[i+1][2]], color='blue', linewidth=2)

    plt.show()


def plot_rrt_3d(tree, path, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    for obstacle in obstacles:
        x, y, z, radius = obstacle  # assuming obstacle is a list [x, y, z, radius]
        phi = np.linspace(0, 2 * np.pi, 100)
        z = np.linspace(0, 1, 100)
        phi, z = np.meshgrid(phi, z)
        x = x + radius * np.cos(phi)
        y = y + radius * np.sin(phi)
        ax.plot_surface(x, y, z, color='red')

    if tree is not None:
        for node in tree.keys():
            ax.scatter(node[0], node[1], node[2], marker='o', color='black', s=2)

        for node, data in tree.items():
            if data['parent'] is not None:
                ax.plot([node[0], data['parent'][0]], [node[1], data['parent'][1]], [node[2], data['parent'][2]], color='green', linewidth=0.4)

    if path is not None:
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], [path[i][2], path[i+1][2]], color='blue', linewidth=2)

    plt.show()