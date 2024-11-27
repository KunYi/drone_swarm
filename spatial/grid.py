"""
Spatial grid system for efficient neighbor search in 3D space.
"""

import numpy as np

class SpatialGrid:
    """Spatial grid system for efficient drone neighbor search in 3D space"""
    def __init__(self, world_size, grid_size=5.0):
        self.grid_size = grid_size
        self.grid_dimensions = int(np.ceil(world_size / grid_size))
        self.grid = {}  # Dictionary to store grid cells

    def _get_grid_key(self, x, y, z):
        """Get the grid cell index for a given position

        Args:
            x: X coordinate
            y: Y coordinate
            z: Z coordinate

        Returns:
            tuple: Grid cell indices (gx, gy, gz)
        """
        gx = int(x / self.grid_size)
        gy = int(y / self.grid_size)
        gz = int(z / self.grid_size)
        return (gx, gy, gz)

    def update_drone_position(self, drone):
        """Update drone position in the grid system

        Args:
            drone: The drone object to update
        """
        key = self._get_grid_key(drone.x, drone.y, drone.z)
        # Clear old position
        for cell in self.grid.values():
            if drone in cell:
                cell.remove(drone)
        # Add new position
        if key not in self.grid:
            self.grid[key] = set()
        self.grid[key].add(drone)

    def get_nearby_drones(self, drone, radius=2):
        """Get nearby drones within specified radius

        Args:
            drone: Target drone
            radius: Search radius in grid cells

        Returns:
            list: List of nearby drones (excluding the target drone)
        """
        nearby = set()
        center = self._get_grid_key(drone.x, drone.y, drone.z)

        # Check surrounding grid cells
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    key = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if key in self.grid:
                        nearby.update(self.grid[key])

        # Remove self from result
        nearby.discard(drone)
        return list(nearby)

    def clear(self):
        """Clear all grid cells"""
        self.grid.clear()

    def add_drone(self, drone):
        """Add a drone to the grid system

        Args:
            drone: The drone object to add
        """
        key = self._get_grid_key(drone.x, drone.y, drone.z)
        if key not in self.grid:
            self.grid[key] = set()
        self.grid[key].add(drone)
