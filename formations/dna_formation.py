"""DNA helix formation calculation module."""
import numpy as np

def calculate_dna_position(i, total_points, center_x, center_y, radius, height, turns):
    """Calculate position for a single point in DNA formation"""
    t = i / total_points
    angle = 2 * np.pi * turns * t

    # Create double helix effect
    if i % 2 == 0:
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
    else:
        x = center_x + radius * np.cos(angle + np.pi)
        y = center_y + radius * np.sin(angle + np.pi)

    z = 3 + height * t  # Start from height 3 meters
    return (x, y, z)

def calculate_dna_formation(num_drones, center_x, center_y, spacing):
    """Calculate positions for DNA helix formation"""
    positions = []
    radius = spacing * 2
    height = 20.0
    turns = 4

    for i in range(num_drones):
        pos = calculate_dna_position(i, num_drones, center_x, center_y,
                                   radius, height, turns)
        positions.append(pos)

    return positions
