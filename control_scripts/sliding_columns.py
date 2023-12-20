
def move_the_column(sliding_column_ids):

    column_velocities = [0.1, 0, 0]

    for i in range(len(sliding_column_ids)):
        p.resetBaseVelocity(sliding_column_ids[i], [column_velocities[i], 0, 0])