"""
Formation manager module for managing drone formations.
"""

from typing import List, Tuple, Dict
import multiprocessing as mp
from multiprocessing import Pool
from concurrent.futures import ThreadPoolExecutor
import numpy as np

from core.config import (
    SAFE_DISTANCE,
    MAX_VELOCITY,
    CONVERGENCE_THRESHOLD
)
from core import Drone
from positioning import AerialBaseStation, GroundStation, PositioningSystem
from formations.phase import FormationPhase
from safety.collision_avoidance import CollisionAvoidance
from spatial import SpatialGrid
from planning.path_planner import PathPlanner

class FormationManager:
    """Manager class for handling drone formation control"""

    def __init__(self, num_drones=125):
        # World configuration
        self.world_size = 50
        self.center_x = self.world_size / 2
        self.center_y = self.world_size / 2
        self.num_drones = num_drones

        # Initialize positioning system
        self._init_positioning_system()

        # Initialize processing pools
        self.pool = Pool(processes=mp.cpu_count())
        self.thread_pool = ThreadPoolExecutor(max_workers=4)

        # Initialize drones and positions
        self._init_ground_positions()
        self._init_drones()

        # Initialize helper systems
        self._init_helper_systems()

        # Movement parameters
        self.max_velocity = MAX_VELOCITY
        self.convergence_threshold = CONVERGENCE_THRESHOLD

    def _init_positioning_system(self):
        """Initialize positioning system with ground and aerial stations"""
        self.ground_station = GroundStation(self.world_size * 0.8, self.world_size * 0.8, 0)
        self.aerial_stations = [
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.2, 0, 0),
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.2, 0, 1),
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.8, 0, 2),
            AerialBaseStation(self.world_size * 0.5, self.world_size * 0.5, 0, 3),
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.8, 0, 4),
        ]
        self.positioning_system = PositioningSystem(self.ground_station, self.aerial_stations)

    def _init_ground_positions(self):
        """Initialize ground positions for drones"""
        spacing = SAFE_DISTANCE * 1.2
        self.ground_positions = []
        rows = int(np.ceil(np.sqrt(self.num_drones)))
        cols = int(np.ceil(self.num_drones / rows))

        for i in range(rows):
            for j in range(cols):
                if len(self.ground_positions) < self.num_drones:
                    x = self.world_size * 0.2 + i * spacing
                    y = self.world_size * 0.2 + j * spacing
                    self.ground_positions.append((x, y, 0))

    def _init_drones(self):
        """Initialize drone objects"""
        self.drones = []
        for i, pos in enumerate(self.ground_positions):
            drone = Drone(pos[0], pos[1], pos[2],
                        pos[0], pos[1], pos[2],
                        i,
                        is_aerial_station=False)
            self.drones.append(drone)
        self.target_positions = self.ground_positions.copy()

    def _init_helper_systems(self):
        """Initialize helper systems like collision avoidance and path planning"""
        self.spatial_grid = SpatialGrid(self.world_size)
        self.collision_avoidance = CollisionAvoidance(
            min_distance=SAFE_DISTANCE,
            max_speed=MAX_VELOCITY,
            boundary_margin=2.0,
            world_size=self.world_size
        )
        self.path_planner = PathPlanner(max_speed=MAX_VELOCITY)

    def get_drone_positions(self) -> List[Tuple[float, float, float]]:
        """Get current positions of all drones"""
        return [(drone.x, drone.y, drone.z) for drone in self.drones]

    def get_target_positions(self) -> List[Tuple[float, float, float]]:
        """Get target positions for all drones"""
        return self.target_positions

    def _calculate_velocities(self, drone: Drone, nearby_drones: List[Drone]) -> Tuple[float, float, float]:
        """Calculate velocities for a drone considering obstacles and target

        Args:
            drone: The drone to calculate velocities for
            nearby_drones: List of nearby drones to consider for collision avoidance

        Returns:
            Tuple of (vx, vy, vz) velocities
        """
        # Get target position for this drone
        target = self.target_positions[drone.id]

        # Update drone's target
        drone.target_x = target[0]
        drone.target_y = target[1]
        drone.target_z = target[2]

        # Calculate base velocity vector towards target
        dx = target[0] - drone.x
        dy = target[1] - drone.y
        dz = target[2] - drone.z

        # Normalize and scale by max velocity
        distance = np.sqrt(dx*dx + dy*dy + dz*dz)
        if distance > self.convergence_threshold:
            scale = min(self.max_velocity, distance) / distance if distance > 0 else 0
            vx = dx * scale
            vy = dy * scale
            vz = dz * scale
        else:
            return 0, 0, 0

        # Get avoidance velocity from drone's built-in collision avoidance
        if nearby_drones:
            avoid_vx, avoid_vy, avoid_vz = drone.calculate_avoidance_velocity(nearby_drones)
            # Blend avoidance and target velocities
            vx = 0.7 * vx + 0.3 * avoid_vx
            vy = 0.7 * vy + 0.3 * avoid_vy
            vz = 0.7 * vz + 0.3 * avoid_vz

        # Ensure velocity limits
        speed = np.sqrt(vx*vx + vy*vy + vz*vz)
        if speed > self.max_velocity:
            scale = self.max_velocity / speed
            vx *= scale
            vy *= scale
            vz *= scale

        return vx, vy, vz

    def update_drone_positions(self, dt: float):
        """Update positions of all drones

        Args:
            dt: Time step in seconds
        """
        # Update spatial grid with current drone positions
        self.spatial_grid.clear()  # Clear old positions
        for drone in self.drones:
            self.spatial_grid.update_drone_position(drone)  # Update each drone's position

        # Process drones in parallel
        def update_drone(drone):
            # Get nearby drones for collision avoidance
            nearby_drones = self.spatial_grid.get_nearby_drones(drone, radius=SAFE_DISTANCE*2)

            # Calculate velocities
            vx, vy, vz = self._calculate_velocities(drone, nearby_drones)

            # Update position
            drone.x += vx * dt
            drone.y += vy * dt
            drone.z += vz * dt

            return drone

        # Use thread pool for parallel updates
        updated_drones = list(self.thread_pool.map(update_drone, self.drones))
        self.drones = updated_drones

    def set_formation_targets(self, target_positions: List[Tuple[float, float, float]]):
        """Set new target positions for formation

        Args:
            target_positions: List of (x, y, z) target positions
        """
        if len(target_positions) != len(self.drones):
            raise ValueError("Number of target positions must match number of drones")
        self.target_positions = target_positions.copy()

    def is_formation_complete(self) -> bool:
        """Check if all drones have reached their targets

        Returns:
            True if formation is complete, False otherwise
        """
        for drone, target in zip(self.drones, self.target_positions):
            dx = target[0] - drone.x
            dy = target[1] - drone.y
            dz = target[2] - drone.z
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            if distance > self.convergence_threshold:
                return False
        return True
