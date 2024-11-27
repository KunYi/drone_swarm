"""
Distributed formation controller for large-scale drone swarms.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

from .base import BaseController
from core.drone import Drone
from positioning.stations import GroundStation, AerialBaseStation
from positioning.system import PositioningSystem

REGION_SIZE = 10.0
MAX_DRONES_PER_REGION = 100

class Region:
    """Region for distributed control"""
    
    def __init__(self, region_id, x, y, z, size, controller):
        """Initialize region
        
        Args:
            region_id: Unique identifier for this region
            x: X coordinate of the region's corner
            y: Y coordinate of the region's corner
            z: Z coordinate of the region's corner
            size: Size of the region
            controller: DistributedController instance
        """
        self.id = region_id
        self.x = x
        self.y = y
        self.z = z
        self.size = size
        self.drones = []
        self.controller = controller
        self.world_size = controller.world_size
        
    def add_drone(self, drone):
        """Add a drone to this region"""
        self.drones.append(drone)
        
    def update(self):
        """Update region state"""
        # Update drone positions within this region
        if not self.drones:
            return
            
        # Calculate local target positions
        cost_matrix = np.zeros((len(self.drones), len(self.drones)))
        for i, drone in enumerate(self.drones):
            for j, other_drone in enumerate(self.drones):
                cost_matrix[i, j] = np.sqrt(
                    (drone.x - other_drone.x)**2 +
                    (drone.y - other_drone.y)**2 +
                    (drone.z - other_drone.z)**2
                )
        
        # Optimize drone assignments
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        # Update drone targets
        for drone_idx, target_idx in zip(row_ind, col_ind):
            tx, ty, tz = self.drones[target_idx].x, self.drones[target_idx].y, self.drones[target_idx].z
            self.drones[drone_idx].target_x = tx
            self.drones[drone_idx].target_y = ty
            self.drones[drone_idx].target_z = tz
            
    def get_neighbor_regions(self):
        """Get neighboring regions.
        
        Returns:
            List of neighboring Region objects.
        """
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor_x = self.x + dx * self.size
                neighbor_y = self.y + dy * self.size
                if 0 <= neighbor_x < self.controller.world_size and 0 <= neighbor_y < self.controller.world_size:
                    region_key = (neighbor_x, neighbor_y)
                    if region_key in self.controller.regions:
                        neighbors.append(self.controller.regions[region_key])
        return neighbors

class DistributedController(BaseController):
    """Distributed controller for large-scale drone formations"""
    
    def __init__(self, num_drones=125, world_size=100.0):
        """Initialize distributed controller.
        
        Args:
            num_drones: Number of drones to control
            world_size: Size of the world
        """
        super().__init__(num_drones, world_size)
        
        # Initialize regions
        self.region_size = REGION_SIZE
        self.max_drones_per_region = MAX_DRONES_PER_REGION
        self.regions = self._init_regions()
        
        # Initialize stations and positioning
        self._init_stations()
        
        # Initialize drones
        self.drones = []
        self._init_drones()
        
        # Distribute drones to regions
        self._assign_drones_to_regions()
    
    def _init_regions(self):
        """Initialize regions for distributed control.
        
        Returns:
            dict: Dictionary of Region objects keyed by (x, y) coordinates
        """
        regions = {}
        num_regions = int(np.ceil(self.world_size / self.region_size))
        
        region_id = 0
        for i in range(num_regions):
            for j in range(num_regions):
                x = i * self.region_size
                y = j * self.region_size
                region = Region(
                    region_id=region_id,
                    x=x,
                    y=y,
                    z=0,
                    size=self.region_size,
                    controller=self
                )
                regions[(x, y)] = region
                region_id += 1
        
        return regions
        
    def _init_stations(self):
        """Initialize ground and aerial stations"""
        # Ground station
        self.ground_station = GroundStation(
            self.world_size * 0.8,
            self.world_size * 0.8,
            0
        )
        
        # Aerial stations - one per region
        self.aerial_stations = []
        for i, region in enumerate(self.regions.values()):
            x = region.x + region.size / 2
            y = region.y + region.size / 2
            z = 0  # Start at ground level
            station = AerialBaseStation(x, y, z, i)
            self.aerial_stations.append(station)
        
        # Initialize positioning system
        self.positioning_system = PositioningSystem(
            self.ground_station,
            self.aerial_stations
        )
    
    def _init_drones(self):
        """Initialize drone objects with random initial positions"""
        for i in range(self.num_drones):
            x = np.random.uniform(0, self.world_size)
            y = np.random.uniform(0, self.world_size)
            z = 0  # Start at ground level
            drone = Drone(x, y, z, x, y, z, i)
            self.drones.append(drone)
    
    def update_formation(self):
        """Update formation state and positions"""
        # Update drone assignments to regions
        self._assign_drones_to_regions()
        
        # Update each region
        for region in self.regions.values():
            region.update()
            
        # Update drone positions
        self.update_positions(self.drones, None)  # Target positions handled by regions
        
    def _assign_drones_to_regions(self):
        """Assign drones to regions based on their positions"""
        # Clear current assignments
        for region in self.regions.values():
            region.drones.clear()
            
        # Assign drones to regions
        for drone in self.drones:
            # Skip drones outside world bounds
            if (drone.x < 0 or drone.x >= self.world_size or
                drone.y < 0 or drone.y >= self.world_size or
                drone.z < 0 or drone.z >= self.world_size):
                continue
                
            region_x = int(drone.x / self.region_size) * self.region_size
            region_y = int(drone.y / self.region_size) * self.region_size
            region_key = (region_x, region_y)
            
            if region_key in self.regions:
                self.regions[region_key].add_drone(drone)
                
    def _get_region_for_position(self, x, y, z):
        """Get region for a given position.
        
        Args:
            x: X coordinate
            y: Y coordinate
            z: Z coordinate
            
        Returns:
            Region object or None if position is out of bounds
        """
        if not (0 <= x < self.world_size and
                0 <= y < self.world_size and
                0 <= z < self.world_size):
            return None
            
        region_x = int(x / self.region_size) * self.region_size
        region_y = int(y / self.region_size) * self.region_size
        
        region_key = (region_x, region_y)
        return self.regions.get(region_key)
