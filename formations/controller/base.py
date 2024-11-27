"""
Base controller module for drone formation control.
Provides core functionality for formation control systems.
"""

import time
import multiprocessing as mp
from multiprocessing import Pool
import numpy as np

from core.config import (
    SAFE_DISTANCE,
    MAX_VELOCITY,
    CONVERGENCE_THRESHOLD,
    MAX_THREADS
)
from positioning.system import PositioningSystem
from safety.collision_avoidance import CollisionAvoidance
from spatial.grid import SpatialGrid
from planning.path import PathPlanner

class BaseController:
    """Base controller class with common functionality"""
    
    def __init__(self, num_drones=125, world_size=100.0):
        """Initialize base controller
        
        Args:
            num_drones: Number of drones to control
            world_size: Size of the world space
        """
        self.num_drones = num_drones
        self.world_size = world_size
        self.center_x = self.world_size / 2
        self.center_y = self.world_size / 2
        
        # Initialize positioning system
        self.positioning_system = None  # Will be set by child class
        
        # Initialize spatial grid
        self.spatial_grid = SpatialGrid(
            world_size=self.world_size,
            grid_size=5.0  # Grid size of 5 meters
        )
        self.path_planner = PathPlanner()
        
        # Initialize convergence parameters
        self.convergence_threshold = CONVERGENCE_THRESHOLD
        
        # Time tracking
        self.start_time = time.time()
        self.last_update_time = self.start_time
        
        # Initialize thread pool
        self.pool = None
        self._init_thread_pool()
        
    def __del__(self):
        """Clean up resources."""
        if self.pool:
            self.pool.terminate()
            self.pool.join()
            
    def _init_thread_pool(self):
        """Initialize thread pool if needed."""
        if self.pool is None:
            self.pool = Pool(processes=min(MAX_THREADS, self.num_drones))
            
    def update_positions(self, drones, target_positions=None):
        """Update drone positions."""
        # Update spatial grid
        for drone in drones:
            self.spatial_grid.update_drone_position(drone)
            
        # Update each drone's position
        for drone in drones:
            nearby_drones = self.spatial_grid.get_nearby_drones(drone)
            drone.update_position(nearby_drones)
    
    def check_convergence(self, drones):
        """Check if all drones have reached their targets
        
        Args:
            drones: List of drones to check
            
        Returns:
            bool: True if all drones have converged
        """
        return all(drone.distance_to_target() < self.convergence_threshold 
                  for drone in drones)
    
    def cleanup(self):
        """Cleanup resources"""
        if self.pool:
            self.pool.close()
            self.pool.join()
