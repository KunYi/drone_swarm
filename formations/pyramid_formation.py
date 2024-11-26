"""Pyramid formation calculation module."""
import numpy as np

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
    total_layers = 5  # Start with 5 layers

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
