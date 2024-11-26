"""Ground formation calculation module."""
import numpy as np

def calculate_ground_formation(num_drones, center_x, center_y, spacing):
    """Calculate positions for ground grid formation"""
    positions = []
    side_length = int(np.ceil(np.sqrt(num_drones)))

    for i in range(num_drones):
        row = i // side_length
        col = i % side_length
        x = center_x + (col - (side_length - 1) / 2) * spacing
        y = center_y + (row - (side_length - 1) / 2) * spacing
        z = 3.0  # Hover height
        positions.append((x, y, z))

    return positions
