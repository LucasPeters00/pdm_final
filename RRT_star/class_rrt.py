import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RRTstar():
    def __init__(self, start, goal, obstacles, step_size, max_iter):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = {tuple(start): {'parent': None, 'cost': 0}}
        self.best_path = None
        self.best_cost = np.inf

    def rrt_star_algorithm(self):
        for i in range(self.max_iter):
            if i % (self.max_iter // 50) == 0:
                print(f"Progress: {i / self.max_iter * 100}%")

            random_point = self.generate_random_point()

            nearest_node = self.find_nearest_node(random_point)

            new_node_position = self.generate_new_node(nearest_node, random_point)

            if self.check_collision(nearest_node, new_node_position):
                continue

            neighbors = self.find_neighbors(new_node_position)

            min_cost_neighbor = self.find_min_cost_neighbor(neighbors)

            new_node = self.add_new_node(min_cost_neighbor, nearest_node, new_node_position)

            self.rewire_tree(neighbors, new_node)

            if self.check_goal_reached(new_node):
                self.update_best_path(new_node)
                self.update_search_radius(i)

        return self.best_path, self.tree

    def generate_random_point(self):
        goal_sampling_rate = 0.05
        if np.random.random() < goal_sampling_rate:
            return self.goal
        else:
            x_min, y_min, z_min = 0, 0, 0
            x_max, y_max, z_max = 4, 6, 1.5
            return np.array([np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max), np.random.uniform(z_min, z_max)])

    def find_nearest_node(self, point):
        distances = [np.linalg.norm(np.array(node) - point) for node in self.tree.keys()]
        nearest_node_index = np.argmin(distances)
        return list(self.tree.keys())[nearest_node_index]

    def generate_new_node(self, nearest_node, random_point):
        direction_vector = random_point - nearest_node
        normalized_direction = direction_vector / np.linalg.norm(direction_vector)
        return tuple(nearest_node + self.step_size * normalized_direction)

    def check_collision(self, nearest_node, new_node_position):
        collision = False
        for column in self.obstacles:
            new_node_array = np.array(new_node_position)
            column_center = np.array(column[:3])
            distance = np.linalg.norm(new_node_array - column_center)
            if distance < column[3]:
                collision = True
                break
        return collision

    def find_neighbors(self, new_node_position):
        neighbors = [node for node in self.tree.keys() if np.linalg.norm(np.array(new_node_position) - np.array(node)) < self.obstacles[0][3]]
        return neighbors

    def find_min_cost_neighbor(self, neighbors):
        costs = [self.tree[neighbor]['cost'] + np.linalg.norm(np.array(neighbors) - np.array(new_node_position)) for neighbor in neighbors]
        min_cost_index = np.argmin(costs)
        return neighbors[min_cost_index]

    def add_new_node(self, min_cost_neighbor, nearest_node, new_node_position):
        parent_cost = self.tree[min_cost_neighbor]['cost']
        distance_to_parent = np.linalg.norm(np.array(min_cost_neighbor) - np.array(new_node_position))
        new_node_cost = parent_cost + distance_to_parent
        new_node = tuple(new_node_position)
        self.tree[new_node] = {'parent': min_cost_neighbor, 'cost': new_node_cost}
        return new_node

    def rewire_tree(self, neighbors, new_node):
        for neighbor in neighbors:
            distance_to_neighbor = np.linalg.norm(np.array(new_node) - np.array(neighbor))
            potential_cost = self.tree[new_node]['cost'] + distance_to_neighbor
            if potential_cost < self.tree[neighbor]['cost']:
                self.tree[neighbor] = {'parent': new_node, 'cost': potential_cost}

    def check_goal_reached(self, new_node):
        distance_to_goal = np.linalg.norm(np.array(new_node) - np.array(self.goal))
        return distance_to_goal < self.step_size

    def update_best_path(self, new_node):
        goal_cost = self.tree[new_node]['cost'] + np.linalg.norm(np.array(new_node) - np.array(self.goal))
        if goal_cost < self.best_cost:
            self.best_cost = goal_cost
            self.tree[tuple(self.goal)] = {'parent': new_node, 'cost': goal_cost}

            path = []
            current_node = tuple(self.goal)
            while current_node is not None:
                path.append(current_node)
                current_node = self.tree[current_node]['parent']
            path.reverse()
            self.best_path = path

    def update_search_radius(self, iteration):
        gamma = 2.5
        d = 3
        r = gamma * ((np.log(iteration + 1) / (iteration + 1))**(1 / (d + 1)))
        self.step_size = min(r, self.step_size)


def plot_rrt(tree, path, obstacles):
    fig, axs = plt.subplots(1, 2)
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
        x, y, z, radius = obstacle
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