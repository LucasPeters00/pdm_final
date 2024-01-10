import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class IRRT:
    def __init__(self, start, goal, obstacles, step_size, max_iter, gamma_kf):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.gamma_kf = gamma_kf
        self.tree = {tuple(start): {'parent': None, 'cost': 0}}
        self.best_path = None
        self.best_cost = np.inf
        #For frawing the ellipse
        self.C = np.zeros([3,3])
        self.L = np.zeros([3,3])
        self.xcenter = np.zeros(3)

 #==============================================================================
#informed RRT star algorithm
        
    def generate_random_point(self):
        goal_sampling_rate = 0.05
        xstart = self.start
        xgoal = self.goal
        cmax = self.best_cost
        
        # If the best cost (cmax) is finite, it means the algorithm has found a feasible path from the start to the goal. 
        # In this case, the function samples a random point from an ellipsoidal region in the space, centered between the start and goal. 
        # The ellipsoid is oriented based on a rotation matrix (C) and scaled using a diagonal matrix (L). 
        if cmax < np.inf:
            cmin = self.getDist(xgoal, xstart)
            xcenter = np.array([(xgoal[0] + xstart[0]) / 2, (xgoal[1] + xstart[1]) / 2, (xgoal[2] + xstart[2]) / 2])
            C = self.RotationToWorldFrame()
            r = np.zeros(3)
            r[0] = cmax / 2
            for i in range(1,3):
                r[i] = np.sqrt(cmax**2 - cmin**2) / 2
            L = np.diag(r)
            xball = self.SampleUnitBall()
            xrand = C@L@xball + xcenter
            self.C = C
            self.xcenter = xcenter
            self.L = L
            return xrand
        else:
            if np.random.random() < goal_sampling_rate:
                return self.goal
            else:
                x_min, y_min, z_min = 0, 0, 0
                x_max, y_max, z_max = 4, 6, 1.5
                return np.array([np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max), np.random.uniform(z_min, z_max)])

    def SampleUnitBall(self):
        # uniform sampling in spherical coordinate system in 3D
        # sample radius
        r = np.random.uniform(0.0, 1.0)
        theta = np.random.uniform(0, np.pi)
        phi = np.random.uniform(0, 2 * np.pi)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return np.array([x,y,z])   

    def RotationToWorldFrame(self):
        # S0(n): such that the xstart and xgoal are the center points
        d = self.getDist(self.start, self.goal)
        xstart, xgoal = np.array(self.start), np.array(self.goal)
        a1 = (xgoal - xstart) / d
        M = np.outer(a1,[1,0,0])
        U, S, V = np.linalg.svd(M)
        C = U@np.diag([1, 1, np.linalg.det(U)*np.linalg.det(V)])@V.T
        return C
    
    def getDist(self, pos1, pos2):
        return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))
#==============================================================================
    #For drawing the ellipse
    #checken of we dit willen gebruiken
    def CreateUnitSphere(self):
        phi = np.linspace(0,2*np.pi, 256).reshape(256, 1) # the angle of the projection in the xy-plane
        theta = np.linspace(0, np.pi, 256).reshape(-1, 256) # the angle from the polar axis, ie the polar angle
        radius = 1

        # Transformation formulae for a spherical coordinate system.
        x = radius*np.sin(theta)*np.cos(phi)
        y = radius*np.sin(theta)*np.sin(phi)
        z = radius*np.cos(theta)
        return (x, y, z)

    def draw_ellipsoid(self, ax):
        (xs, ys, zs) = self.CreateUnitSphere()
        pts = np.array([xs, ys, zs])
        pts_in_world_frame = self.C@self.L@pts + self.xcenter
        ax.plot_surface(pts_in_world_frame[0], pts_in_world_frame[1], pts_in_world_frame[2], alpha=0.05, color="g")
    
#==============================================================================
    def find_nearest_node(self, random_point):
        distances = [np.linalg.norm(np.array(node) - random_point) for node in self.tree.keys()]
        nearest_node_index = np.argmin(distances)
        return list(self.tree.keys())[nearest_node_index]

    def create_new_node(self, nearest_node, random_point):
        direction_vector = random_point - nearest_node
        normalized_direction = direction_vector / np.linalg.norm(direction_vector)
        new_node_position = nearest_node + self.step_size * normalized_direction
        return tuple(new_node_position)

    def check_collision(self, new_node_position):
        for column in self.obstacles:
            new_node_array = np.array(new_node_position[:2])
            column_center = np.array(column[:2])
            distance = np.linalg.norm(new_node_array - column_center)
            if distance < column[3]:
                return True
        return False

    def find_neighbors(self, new_node_position):
        card_v = len(self.tree)
        dimension = len(new_node_position)
        min_radius = 0.1  # Minimum radius value to avoid very small values
        radius = max(min_radius, self.gamma_kf * (np.log(card_v) / card_v) ** (1 / (dimension + 1))) # Use of Solovey et al. (2020) formula

        neighbors = []
        for node in self.tree.keys():
            distance = np.linalg.norm(np.array(node) - np.array(new_node_position))
            if distance < radius:
                if not self.check_collision_line(node, new_node_position):
                    neighbors.append(node)

        return neighbors
        
    def find_min_cost_neighbor(self, neighbors):
        costs = [
            self.tree[neighbor]['cost'] + np.linalg.norm(np.array(neighbor) - np.array(self.tree[neighbor]['parent']))
            if self.tree[neighbors[0]]['parent'] is not None
         else np.inf
            for neighbor in neighbors
        ]
        min_cost_index = np.argmin(costs)
        return neighbors[min_cost_index]

    def add_new_node(self, min_cost_neighbor, new_node_position):
        parent_cost = self.tree[min_cost_neighbor]['cost']
        distance_to_parent = np.linalg.norm(np.array(min_cost_neighbor) - np.array(new_node_position))
        new_node_cost = parent_cost + distance_to_parent
        new_node = tuple(new_node_position)
        if new_node is not None:
            self.tree[new_node] = {'parent': min_cost_neighbor, 'cost': new_node_cost}
        return new_node

    def check_collision_line(self, start_point, end_point):
        start = np.array(start_point)[:2]
        end = np.array(end_point)[:2]

        for obstacle in self.obstacles:
            obstacle_center = np.array(obstacle[:2])
            radius = obstacle[3]

            line_vector = end - start
            obstacle_vector = obstacle_center - start

            t = np.dot(obstacle_vector, line_vector) / np.dot(line_vector, line_vector)

            closest_point = start + np.clip(t, 0, 1) * line_vector

            if np.linalg.norm(obstacle_center - closest_point) < radius:
                return True

        return False

    def rewire_tree(self, neighbors, new_node):
        for neighbor in neighbors:
            distance_to_neighbor = np.linalg.norm(np.array(new_node) - np.array(neighbor))

            # Voeg botsingscontrole toe voordat je doorgaat
            potential_node_position = np.array(neighbor)
            if self.check_collision_line(new_node, potential_node_position):
                continue  # Sla deze buur over als er een botsing is

            potential_cost = self.tree[new_node]['cost'] + distance_to_neighbor
            if potential_cost < self.tree[neighbor]['cost']:
                self.tree[neighbor] = {'parent': new_node, 'cost': potential_cost}

    def check_goal_reached(self, new_node):
        distance_to_goal = np.linalg.norm(np.array(new_node) - np.array(self.goal))
        return distance_to_goal < self.step_size

    def rrt_star_algorithm(self):
        for i in range(self.max_iter):
            if i % (self.max_iter // 50) == 0:
                print(f"Progress: {i / self.max_iter * 100}%")

            random_point = self.generate_random_point()

            nearest_node = self.find_nearest_node(random_point)

            new_node_position = self.create_new_node(nearest_node, random_point)

            if self.check_collision(new_node_position):
                continue

            neighbors = self.find_neighbors(new_node_position)

            if not neighbors:
                continue

            min_cost_neighbor = self.find_min_cost_neighbor(neighbors)

            new_node = self.add_new_node(min_cost_neighbor, new_node_position)

            self.rewire_tree(neighbors, new_node)

            if self.check_goal_reached(new_node):
                distance_to_goal = np.linalg.norm(np.array(new_node) - np.array(self.goal))
                goal_cost = self.tree[new_node]['cost'] + distance_to_goal
                if goal_cost < self.best_cost:
                    self.best_cost = goal_cost
                    self.tree[tuple(self.goal)] = {'parent': new_node, 'cost': goal_cost}
                    # print("Path found or better path found")

                    # Construct the path
                    path = []
                    current_node = tuple(self.goal)
                    while current_node is not None:
                        path.append(current_node)
                        current_node = self.tree[current_node]['parent']
                    path.reverse()  # Reverse the path to go from start to goal

                    self.best_path = path

        return self.best_path, self.tree

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

    