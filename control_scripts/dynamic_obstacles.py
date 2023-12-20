

def obstacle_avoidance_constraint(x, u, dynamic_obstacles, safety_margin):
    constraints = []

    for obstacle in dynamic_obstacles:
        relative_position = x[:3] - obstacle.position
        relative_velocity = x[3:6] - obstacle.velocity
        distance_to_obstacle = cp.norm(relative_position)

        # Voeg een constraint toe op basis van de dynamische obstakels
        constraints += [distance_to_obstacle >= obstacle.radius + safety_margin + cp.norm(relative_velocity) * time_horizon]

    return constraints

# Data Required of dynamic_obstacles to use in the mpc controller as constraint:

### Dynamic Obstacle Information:
# You need information about the dynamic obstacles, including their initial positions, radii, and motion models (if any). 
# Additionally, you need to predict their positions within the MPC prediction horizon.

# Spherical obstacles prior information:
# Obstacles are defined as spheres with a radius and a position.
# The position of the obstacle is defined in the world frame.
# The radius of the obstacle is defined in the world frame.
# The velocity of the obstacle is defined in the world frame.
# Obstacles have constant velocity
# Aangenomen dynamische obstakels

import numpy as np

class DynamicObstacle:
    def __init__(self, position, velocity, radius):
        self.position = position
        self.velocity = velocity
        self.radius = radius

# Initialisatie van dynamische obstakels
dynamic_obstacle_1 = DynamicObstacle(position=np.array([5, 5, 0]), velocity=np.array([1, 0, 0]), radius=1.0)
dynamic_obstacle_2 = DynamicObstacle(position=np.array([8, 8, 0]), velocity=np.array([0, 1, 0]), radius=1.5)

dynamic_obstacles = [dynamic_obstacle_1, dynamic_obstacle_2]

# Initialisatie van andere parameters
x_init = np.array([0, 0, 0, 0, 0, 0])
waypoint = np.array([10, 10, 0])
A = np.eye(6)
B = np.eye(3)
Q = np.eye(6)
R = np.eye(3)
horizon = 10
max_velocity = 5.0
max_acceleration = 2.0

# MPC met CVO-functionaliteit
u, x_next = mpc_control_drone_with_cvo(x_init, waypoint, A, B, Q, R, horizon, max_velocity, max_acceleration, dynamic_obstacles)

# URDF file inladen om columns te visualiseren



###  Algorithm Overview:

# Given the constant velocity of obstacles, the GVO algorithm typically involves predicting the future positions of 
# both the moving entity (e.g., a drone) and the obstacles over a short time horizon.

# The GVO method then defines a region in the velocity space of the moving entity that should be avoided to prevent 
# collisions with the predicted future positions of the obstacles.

# If the desired velocity of the moving entity lies outside the GVO region, the trajectory is considered safe; 
# otherwise, adjustments are made to the trajectory to avoid the unsafe region.

###  Predict Future Positions:
#     Predict the future positions of the drone and obstacles based on their constant velocities.

# Compute GVO:
#     Calculate the GVO by considering the constraints on the velocity of the moving entity and the predicted future positions of the obstacles.

# Evaluate Safety:
#     Check if the desired velocity of the drone lies within the GVO region. If it does, the trajectory needs to be adjusted to avoid potential collisions.

# Adjust Trajectory:
#     If necessary, modify the trajectory to ensure that the drone avoids the GVO region, thus avoiding collisions with the moving obstacles.

def mpc_control_drone_with_cvo(x_init, waypoint, A, B, Q, R, horizon, max_velocity, max_acceleration, dynamic_obstacles):
    # ... (overige functiecode blijft ongewijzigd)

    for t in range(horizon):
        # CVO-gerelateerde beperkingen
        for obstacle in dynamic_obstacles:
            relative_position = x[:3, t] - obstacle.position
            relative_velocity = x[3:6, t] - obstacle.velocity
            distance_to_obstacle = cp.norm(relative_position)

            # Update de afstandsbeperking op basis van de CVO-logica
            constraints += distance_to_obstacle >= obstacle.radius + safety_margin + cp.norm(relative_velocity) * time_horizon


# losse functie die door de main heen loopt 
# Als input wordt gebruikt voor de mpc_control_drone
# Functie die een constraint opstelt voor een dynamisch obstakel
