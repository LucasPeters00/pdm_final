import numpy as np
import cvxpy as cp

class DynamicObstacle:
    def __init__(self, position, velocity, radius):
        self.position = position
        self.velocity = velocity
        self.radius = radius

# Dummy dynamic obstacles
obstacle1 = DynamicObstacle(position=np.array([5.0, 0.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)
obstacle2 = DynamicObstacle(position=np.array([0.0, 3.0, 0.8]), velocity=np.array([1.0, 0.0, 0.0]), radius=0.15)

# List of dynamic obstacles
dynamic_obstacles = [obstacle1, obstacle2]

def obstacle_avoidance_constraint(x, u, dynamic_obstacles, safety_margin):
    constraints = []

    for obstacle in dynamic_obstacles:
        relative_position = x[:3] - obstacle.position
        relative_velocity = x[3:6] - obstacle.velocity
        distance_to_obstacle = cp.norm(relative_position)

        # Voeg een constraint toe op basis van de dynamische obstakels
        constraints += [distance_to_obstacle >= obstacle.radius + safety_margin + cp.norm(relative_velocity) * time_horizon]

    return constraints

