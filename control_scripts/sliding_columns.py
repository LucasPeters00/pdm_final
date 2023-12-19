import pybullet as p

column_position = [0.5, 0.5, 0.8]

def sliding_column():
    
    p.loadURDF("urdf_files/column.urdf", column_positions, p.getQuaternionFromEuler([0, 0, 0]))


    
    return 

#HIER EEN KLEIN BEGINNETJE GEMAAKT, HEB EEN IDEE MET GVO OF ANDERE COLLISION AVOIDANCE ALS CONSTRAINTS IN MPC