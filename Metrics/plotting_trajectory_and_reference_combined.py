import matplotlib.pyplot as plt
import numpy as np


def plot_trajectory_3d(trajectory, path, obstacles, tree):

    trajectory = np.array(trajectory)
    path = np.array(path)
    obstacles = np.array(obstacles)
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color = 'green', label='Actual Trajectory', linewidth=3)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color='yellow', label='Waypoint Trajectory', linestyle='--', linewidth=3)

    for obstacle in obstacles:
        x, y, z, radius = obstacle
        phi = np.linspace(0, 2 * np.pi, 100)
        z = np.linspace(0, 1, 100)
        phi, z = np.meshgrid(phi, z)
        x = x + radius * np.cos(phi)
        y = y + radius * np.sin(phi)
        ax.plot_surface(x, y, z, color='red', alpha = 0.1)

    ax.set_title('3D Trajectory of the static columns, the quadrotor trajectory and the waypoint trajectory')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.legend()
    ax.set_box_aspect([1,1,1])  

    if tree is not None:
        for node in tree.keys():
            ax.scatter(node[0], node[1], node[2], marker='o', color='black', s=0.01)

        for node, data in tree.items():
            if data['parent'] is not None:
                ax.plot([node[0], data['parent'][0]], [node[1], data['parent'][1]], [node[2], data['parent'][2]], color='green', linewidth=0.1, alpha = 0.8, label='RRT* Tree')


    plt.show()

def plot_trajectory_2d(trajectory, path, obstacles, tree):

    trajectory = np.array(trajectory)
    path = np.array(path)
    obstacles = np.array(obstacles)
    fig, ax = plt.subplots(figsize =(6, 10)) 
    ax.plot(trajectory[:, 0], trajectory[:, 1], label='Actual Trajectory')
    ax.plot(path[:, 0], path[:, 1], label='Waypoint Trajectory', linestyle='--')

    for obstacle in obstacles:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[3], color='red', alpha = 0.1)
        ax.add_patch(circle)
    
    if tree is not None:
        for node in tree.keys():
            ax.plot(node[0], node[1], marker='o', color='black', markersize=.01)

        for node, data in tree.items():
            if data['parent'] is not None:
                ax.plot([node[0], data['parent'][0]], [node[1], data['parent'][1]], color='green', linewidth=0.3)

    ax.set_title('2D Projection(X-Y Plane)')
    ax.set_xlabel('X axis')  
    ax.set_ylabel('Y axis') 
    ax.legend()

    plt.show()