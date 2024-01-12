import pybullet as p

def add_obstacles(): #function to add the obstacles for the simulation
    num_columns = 12  
    column_positions = [
        [0, 1, 0.8], [1, 1, 0.8], [2, 1, 0.8],
        [0, 2, 0.8], [1, 2, 0.8], [2, 2, 0.8],
        [0, 3, 0.8], [1, 3, 0.8], [2, 3, 0.8],
        [0, 4, 0.8], [1, 4, 0.8], [2, 4, 0.8]
    ]

    # Add the sliding columns
    sliding_column_ids = []
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [-1.5,1.5,.8], p.getQuaternionFromEuler([0, 0, 0])))
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [3.5,2.5,.8], p.getQuaternionFromEuler([0, 0, 0])))
    sliding_column_ids.append(p.loadURDF("urdf_files/sliding_column.urdf", [-1.5,3.5,.8], p.getQuaternionFromEuler([0, 0, 0])))

    # Add the landing box
    p.loadURDF("urdf_files/landing_box.urdf", [0,0,0.1], p.getQuaternionFromEuler([0, 0, 0]) )

    obstacle_dimensions = []

    for i in range(num_columns): # Add the columns
        p.loadURDF("urdf_files/column.urdf", column_positions[i], p.getQuaternionFromEuler([0, 0, 0]))
        x = column_positions[i][0]
        y = column_positions[i][1]
        z = column_positions[i][2]
        radius = 0.3 #slightly larger then real, to have a safety margin.
        obstacle_dimensions.append([x,y,z,radius])
    
    return obstacle_dimensions , sliding_column_ids

def add_obstacles_rrt_star(): #function to add the obstacles for the RRT* algorithm testing 
    num_columns = 96  
    column_positions = [
        [0, 1, 0.8], [1, 1, 0.8], [2, 1, 0.8], [3, 1, 0.8], [4, 1, 0.8], [5, 1, 0.8], [6, 1, 0.8], [7, 1, 0.8],
        [0, 2, 0.8], [1, 2, 0.8], [2, 2, 0.8], [3, 2, 0.8], [4, 2, 0.8], [5, 2, 0.8], [6, 2, 0.8], [7, 2, 0.8],
        [0, 3, 0.8], [1, 3, 0.8], [2, 3, 0.8], [3, 3, 0.8], [4, 3, 0.8], [5, 3, 0.8], [6, 3, 0.8], [7, 3, 0.8],
        [0, 4, 0.8], [1, 4, 0.8], [2, 4, 0.8], [3, 4, 0.8], [4, 4, 0.8], [5, 4, 0.8], [6, 4, 0.8], [7, 4, 0.8],
        [0, 5, 0.8], [1, 5, 0.8], [2, 5, 0.8], [3, 5, 0.8], [4, 5, 0.8], [5, 5, 0.8], [6, 5, 0.8], [7, 5, 0.8],
        [0, 6, 0.8], [1, 6, 0.8], [2, 6, 0.8], [3, 6, 0.8], [4, 6, 0.8], [5, 6, 0.8], [6, 6, 0.8], [7, 6, 0.8],
        [0, 7, 0.8], [1, 7, 0.8], [2, 7, 0.8], [3, 7, 0.8], [4, 7, 0.8], [5, 7, 0.8], [6, 7, 0.8], [7, 7, 0.8],
        [0, 8, 0.8], [1, 8, 0.8], [2, 8, 0.8], [3, 8, 0.8], [4, 8, 0.8], [5, 8, 0.8], [6, 8, 0.8], [7, 8, 0.8],
        [0, 9, 0.8], [1, 9, 0.8], [2, 9, 0.8], [3, 9, 0.8], [4, 9, 0.8], [5, 9, 0.8], [6, 9, 0.8], [7, 9, 0.8],
        [0, 10, 0.8], [1, 10, 0.8], [2, 10, 0.8], [3, 10, 0.8], [4, 10, 0.8], [5, 10, 0.8], [6, 10, 0.8], [7, 10, 0.8],
        [0, 11, 0.8], [1, 11, 0.8], [2, 11, 0.8], [3, 11, 0.8], [4, 11, 0.8], [5, 11, 0.8], [6, 11, 0.8], [7, 11, 0.8],
        [0, 12, 0.8], [1, 12, 0.8], [2, 12, 0.8], [3, 12, 0.8], [4, 12, 0.8], [5, 12, 0.8], [6, 12, 0.8], [7, 12, 0.8],
    ]
    obstacle_dimensions = []

    for i in range(num_columns):
        x = column_positions[i][0]
        y = column_positions[i][1]
        z = column_positions[i][2]
        radius = 0.3  #slightly larger then real, to have a safety margin.
        obstacle_dimensions.append([x,y,z,radius])
    
    return obstacle_dimensions 

def move_the_column(sliding_column_ids): #function to move the columns
    velocity = 2
    min_x = -1.5  
    max_x = 3  
    for column_id in sliding_column_ids:
        position, _ = p.getBasePositionAndOrientation(column_id)
        if position[0] <= min_x:
            p.resetBaseVelocity(column_id, [velocity, 0, 0])
        elif position[0] >= max_x:
            p.resetBaseVelocity(column_id, [-velocity, 0, 0])

    return sliding_column_ids, velocity