import pybullet as p

def add_obstacles():
    num_columns = 12  
    column_positions = [
        [0, 1, 0.8], [1, 1, 0.8], [2, 1, 0.8],
        [0, 2, 0.8], [1, 2, 0.8], [2, 2, 0.8],
        [0, 3, 0.8], [1, 3, 0.8], [2, 3, 0.8],
        [0, 4, 0.8], [1, 4, 0.8], [2, 4, 0.8]
    ]

    sliding_column_ids = []
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [-1,1.5,.8], p.getQuaternionFromEuler([0, 0, 0])))
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [3,2.5,.8], p.getQuaternionFromEuler([0, 0, 0])))
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [-1,3.5,.8], p.getQuaternionFromEuler([0, 0, 0])))


    p.loadURDF("urdf_files/landing_box.urdf", [0,0,0.1], p.getQuaternionFromEuler([0, 0, 0]) )
    p.loadURDF("urdf_files/landing_box.urdf", [2,5.5,0.1], p.getQuaternionFromEuler([0, 0, 0]) ) 
    obstacle_dimensions = []

    for i in range(num_columns):
        p.loadURDF("urdf_files/column.urdf", column_positions[i], p.getQuaternionFromEuler([0, 0, 0]))

        # Calculate the dimensions
        x = column_positions[i][0]
        y = column_positions[i][1]
        z = column_positions[i][2]

        #slightly larger then real, to have a safety margin.
        radius = 0.3

        # Flatten the tuple before adding it to the list
        obstacle_dimensions.append([x,y,z,radius])
    
    return obstacle_dimensions, sliding_column_ids

def move_the_column(sliding_column_ids):

    velocity = 2
    min_x = -1  
    max_x = 3  

    for column_id in sliding_column_ids:
        position, _ = p.getBasePositionAndOrientation(column_id)
        if position[0] <= min_x:
            p.resetBaseVelocity(column_id, [velocity, 0, 0])
        elif position[0] >= max_x:
            p.resetBaseVelocity(column_id, [-velocity, 0, 0])

    return sliding_column_ids, velocity