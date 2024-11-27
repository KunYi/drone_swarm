"""Collision avoidance module for drone swarm."""
import numpy as np

class CollisionAvoidance:
    def __init__(self, min_distance=1.5, max_speed=0.5, boundary_margin=2.0, world_size=40):
        self.min_distance = min_distance
        self.max_speed = max_speed
        self.boundary_margin = boundary_margin
        self.world_size = world_size

    def calculate_avoidance_velocity(self, drone, nearby_drones):
        """Calculate avoidance velocity for a drone based on nearby drones

        Args:
            drone: The drone to calculate avoidance for
            nearby_drones: List of nearby drones to avoid

        Returns:
            Tuple of (vx, vy, vz) avoidance velocities
        """
        if not nearby_drones:
            return 0, 0, 0

        # Initialize avoidance velocity
        avoid_vx, avoid_vy, avoid_vz = 0, 0, 0

        # Calculate repulsion from each nearby drone
        for other in nearby_drones:
            # Calculate direction vector
            dx = drone.x - other.x
            dy = drone.y - other.y
            dz = drone.z - other.z

            # Calculate distance
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            if distance < 0.0001:  # Avoid division by zero
                continue

            # Only avoid if within minimum distance
            if distance < self.min_distance:
                # Calculate repulsion force (stronger as drones get closer)
                force = (self.min_distance - distance) / self.min_distance

                # Add scaled repulsion vector
                scale = force * self.max_speed / distance
                avoid_vx += dx * scale
                avoid_vy += dy * scale
                avoid_vz += dz * scale

        # Apply boundary avoidance
        avoid_vx += self._calculate_boundary_avoidance(drone.x, 0, self.world_size)
        avoid_vy += self._calculate_boundary_avoidance(drone.y, 0, self.world_size)
        avoid_vz += self._calculate_boundary_avoidance(drone.z, 0, self.world_size)

        # Limit maximum avoidance velocity
        speed = np.sqrt(avoid_vx*avoid_vx + avoid_vy*avoid_vy + avoid_vz*avoid_vz)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            avoid_vx *= scale
            avoid_vy *= scale
            avoid_vz *= scale

        return avoid_vx, avoid_vy, avoid_vz

    def _calculate_boundary_avoidance(self, pos, min_bound, max_bound):
        """Calculate avoidance velocity for world boundaries

        Args:
            pos: Current position
            min_bound: Minimum boundary
            max_bound: Maximum boundary

        Returns:
            Avoidance velocity component
        """
        if pos < min_bound + self.boundary_margin:
            return self.max_speed * (1.0 - (pos - min_bound) / self.boundary_margin)
        elif pos > max_bound - self.boundary_margin:
            return -self.max_speed * (1.0 - (max_bound - pos) / self.boundary_margin)
        return 0

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
