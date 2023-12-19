def control_drone(current_position, waypoint, dt, total_time_per_waypoint):

    t = dt / total_time_per_waypoint

    # Compute the interpolated position
    interpolated_position = [(1 - t) * current + t * target for current, target in zip(current_position, waypoint)]

    return interpolated_position
