import pybullet as p
import numpy as np

# Define the drone class
class Drone:
    def __init__(self, drone_model, start_position, drone_mass, dt):
        self.drone_id = p.loadURDF(drone_model, basePosition=start_position)
        self.mass = drone_mass
        self.position = start_position
        self.velocity = np.zeros(3) # Assume the drone starts at rest
        self.dt = dt

        # Define the state transition matrix and control matrix
        self.A = np.array([[1, 0, 0, dt, 0, 0],
                           [0, 1, 0, 0, dt, 0],
                           [0, 0, 1, 0, 0, dt],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        self.B = np.array([[0.5 * dt**2, 0, 0],
                           [0, 0.5 * dt**2, 0],
                           [0, 0, 0.5 * dt**2],
                           [dt, 0, 0],
                           [0, dt, 0],
                           [0, 0, dt]])
        
        
    # Update the state of the drone
    def update_state(self):
        self.position, _ = p.getBasePositionAndOrientation(self.drone_id)
        self.velocity, _ = p.getBaseVelocity(self.drone_id)
        return np.concatenate((self.position, self.velocity))
    
    # Apply the control input to the drone
    def apply_control(self, control_input):
        force = control_input * self.mass # Calculate force that needs to be applied
        p.applyExternalForce(self.drone_id, -1, force, self.position, p.WORLD_FRAME)  #Apply the force at the drone's position in the world frame


    


