"""
Formation calculation module for drone swarm.
This module contains all the mathematical functions for calculating
different formation positions.
"""

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
        z = 1.0  # Height of 1 meter
        positions.append((x, y, z))

    return positions

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

def calculate_sphere_position(i, total, center_x, center_y, radius):
    """Calculate position for a single point in a sphere formation"""
    # Use golden spiral algorithm for uniform distribution
    golden_ratio = (1 + 5**0.5) / 2
    theta = 2 * np.pi * i / golden_ratio
    phi = np.arccos(1 - 2 * (i + 0.5) / total)

    x = center_x + radius * np.sin(phi) * np.cos(theta)
    y = center_y + radius * np.sin(phi) * np.sin(theta)
    z = 4 + radius * np.cos(phi)  # Start from height 4 meters

    return (x, y, z)

def calculate_sphere_formation(num_drones, center_x, center_y, spacing):
    """Calculate positions for sphere formation"""
    positions = []
    radius = spacing * 3  # Adjust radius based on spacing

    for i in range(num_drones):
        pos = calculate_sphere_position(i, num_drones, center_x, center_y, radius)
        positions.append(pos)

    return positions

def calculate_pyramid_layer(layer, total_layers, center_x, center_y, layer_height=2.0, base_size=12.0):
    """Calculate positions for a pyramid layer"""
    positions = []

    # Calculate current layer size
    layer_ratio = (total_layers - layer) / total_layers
    current_size = base_size * layer_ratio

    # Calculate points for the layer
    points_per_side = max(2, int(4 * layer_ratio))

    if layer == 0:  # Top layer
        positions.append((center_x, center_y, layer * layer_height))
    else:
        # Generate points for each side
        for i in range(points_per_side):
            ratio = i / (points_per_side - 1)
            # Front edge
            x = center_x - current_size/2 + current_size * ratio
            y = center_y - current_size/2
            positions.append((x, y, layer * layer_height))
            # Back edge
            y = center_y + current_size/2
            positions.append((x, y, layer * layer_height))
            # Left edge
            x = center_x - current_size/2
            y = center_y - current_size/2 + current_size * ratio
            positions.append((x, y, layer * layer_height))
            # Right edge
            x = center_x + current_size/2
            positions.append((x, y, layer * layer_height))

    return positions

def calculate_pyramid_formation(num_drones, center_x, center_y):
    """Calculate positions for pyramid formation"""
    positions = []
    total_layers = int(np.sqrt(num_drones))

    # Calculate positions for each layer
    for layer in range(total_layers):
        layer_positions = calculate_pyramid_layer(layer, total_layers, center_x, center_y)
        positions.extend(layer_positions)

    # Add extra layers if needed
    while len(positions) < num_drones:
        extra_layer = calculate_pyramid_layer(total_layers, total_layers + 1,
                                           center_x, center_y)
        positions.extend(extra_layer)
        total_layers += 1

    return positions[:num_drones]  # Ensure exact number of positions

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
