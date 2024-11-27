"""Sphere formation calculation module."""
import numpy as np

def calculate_sphere_position(i, total, center_x, center_y, radius):
    """Calculate position for a single point in a sphere formation"""
    # Use golden spiral algorithm for uniform distribution
    phi = np.pi * (3 - np.sqrt(5))  # Golden angle
    y = 1 - (i / float(total - 1)) * 2  # y goes from 1 to -1
    radius_at_y = np.sqrt(1 - y * y)  # radius at y

    theta = phi * i  # Golden angle increment

    x = center_x + np.cos(theta) * radius_at_y * radius
    y = center_y + y * radius
    z = 8 + np.sin(theta) * radius_at_y * radius
    return (x, y, z)

def calculate_sphere_formation(num_drones, center_x, center_y, spacing):
    """Calculate positions for sphere formation"""
    positions = []
    radius = spacing * 4.5  # Adjust radius based on spacing

    for i in range(num_drones):
        pos = calculate_sphere_position(i, num_drones, center_x, center_y, radius)
        positions.append(pos)

    return positions
