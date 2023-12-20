


p.loadURDF("urdf_files/column.urdf", column_positions[i], p.getQuaternionFromEuler([0, 0, 0]))

def update_sliding_column(column_id, start_position=[0.5,0.5,0.8], velocity=10, max_slide_distance=5, dt):

    new_x = start_position[0] + velocity * dt

    # Reverse direction if the maximum slide distance is reached
    if abs(new_x - start_position[0]) >= max_slide_distance:
        # Reset the elapsed time and change the direction of the velocity
        elapsed_time = 0
        velocity = -velocity

    # Update the column's position
    new_position = [new_x, start_position[1], start_position[2]]
    p.resetBasePositionAndOrientation(column_id, new_position, p.getQuaternionFromEuler([0, 0, 0]))

    return 


#HIER EEN KLEIN BEGINNETJE GEMAAKT, HEB EEN IDEE MET GVO OF ANDERE COLLISION AVOIDANCE ALS CONSTRAINTS IN MPC