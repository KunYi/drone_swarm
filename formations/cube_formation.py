"""Cube formation calculation module."""
import numpy as np

def calculate_cube_formation(num_drones, center_x, center_y, spacing):
    """Calculate positions for cube formation"""
    positions = []
    side_length = int(np.ceil(np.cbrt(num_drones)))
    layer_size = side_length * side_length

    for i in range(num_drones):
        k = i // layer_size  # layer number
        remainder = i % layer_size
        j = remainder // side_length  # row number within layer
        l = remainder % side_length  # column number within row

        x = center_x + (l - (side_length - 1) / 2) * spacing
        y = center_y + (j - (side_length - 1) / 2) * spacing
        z = 3 + k * spacing  # Start from height 3 meters
        positions.append((x, y, z))

    return positions
