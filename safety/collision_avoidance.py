"""Collision avoidance module for drone swarm."""
import numpy as np

class CollisionAvoidance:
    def __init__(self, min_distance=1.5, max_speed=0.5, boundary_margin=2.0, world_size=40):
        self.min_distance = min_distance
        self.max_speed = max_speed
        self.boundary_margin = boundary_margin
        self.world_size = world_size

    def avoid_collisions(self, positions, dt):
        """Apply collision avoidance between drones"""
        new_positions = np.array(positions)

        # Calculate all pairwise distances
        positions_array = np.array(positions)
        distances = np.linalg.norm(positions_array[:, np.newaxis] - positions_array, axis=2)

        # Find pairs that are too close
        close_pairs = np.where(distances < self.min_distance)

        # Apply repulsion for each close pair
        for i, j in zip(*close_pairs):
            if i >= j:  # Avoid double counting
                continue

            # Calculate repulsion vector
            direction = positions_array[i] - positions_array[j]
            distance = np.linalg.norm(direction)
            if distance < 0.0001:  # Avoid division by zero
                direction = np.array([0.01, 0.01, 0.01])
                distance = np.linalg.norm(direction)

            # Calculate repulsion force
            repulsion = direction / distance * (self.min_distance - distance)
            repulsion *= 0.5 * self.max_speed * dt

            # Apply repulsion
            new_positions[i] += repulsion
            new_positions[j] -= repulsion

        # Ensure drones stay within world boundaries
        new_positions = np.clip(new_positions,
                              self.boundary_margin,
                              self.world_size - self.boundary_margin)

        return [tuple(pos) for pos in new_positions]
