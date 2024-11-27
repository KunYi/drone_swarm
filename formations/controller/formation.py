"""
Formation controller module for managing drone formations.
"""

from enum import Enum, auto
import numpy as np
import time
from matplotlib import pyplot as plt

from .base import BaseController
from core.drone import Drone
from positioning.stations import GroundStation, AerialBaseStation
from positioning.system import PositioningSystem
from formations.phase import FormationPhase
from formations.patterns import (
    calculate_ground_formation,
    calculate_cube_formation,
    calculate_sphere_formation,
    calculate_pyramid_formation,
    calculate_dna_formation
)

from core.config import SAFE_DISTANCE

# Configuration constants
FORMATION_SPACING = SAFE_DISTANCE * 1.5  # meters
CONVERGENCE_THRESHOLD = 0.1  # meters

class FormationController(BaseController):
    """Controller for managing drone formations and transitions"""
    
    def __init__(self, num_drones=125, world_size=100.0):
        """Initialize formation controller.
        
        Args:
            num_drones: Number of drones to control
            world_size: Size of the world space
        """
        super().__init__(num_drones, world_size)
        
        # Formation phase tracking
        self.current_phase = FormationPhase.GROUND
        self.phase_start_time = time.time()
        self.phase_durations = {
            FormationPhase.GROUND: 10.0,
            FormationPhase.PREPARE: 5.0,
            FormationPhase.TRANSITION: 15.0,
            FormationPhase.CUBE: 20.0,
            FormationPhase.SPHERE: 20.0,
            FormationPhase.PYRAMID: 20.0,
            FormationPhase.DNA: 30.0,
            FormationPhase.LANDING: 10.0
        }
        
        # Formation parameters
        self.formation_spacing = FORMATION_SPACING  # meters between drones
        self.target_positions = None
        
        # Initialize stations
        self._init_stations()
        
        # Initialize drones
        self.drones = []
        self.ground_positions = self._calculate_initial_positions()
        self._init_drones()
        
        # Initialize formation sequence
        self.formation_sequence = [
            FormationPhase.GROUND,
            FormationPhase.PREPARE,
            FormationPhase.TRANSITION,
            FormationPhase.CUBE,
            FormationPhase.SPHERE,
            FormationPhase.PYRAMID,
            FormationPhase.DNA,
            FormationPhase.LANDING
        ]
        self.current_formation_index = 0
        
        # Visualization
        self._init_visualization()
    
    def _init_stations(self):
        """Initialize ground and aerial stations"""
        # Ground station at the edge of the field
        self.ground_station = GroundStation(
            self.world_size * 0.8,
            self.world_size * 0.8,
            0
        )
        
        # Aerial stations at strategic positions
        self.aerial_stations = [
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.2, 0, 0),  # Front left
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.2, 0, 1),  # Front right
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.8, 0, 2),  # Back left
            AerialBaseStation(self.world_size * 0.5, self.world_size * 0.5, 0, 3),  # Center
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.8, 0, 4),  # Back right
        ]
        
        # Initialize positioning system
        self.positioning_system = PositioningSystem(
            self.ground_station,
            self.aerial_stations
        )
    
    def _calculate_initial_positions(self):
        """Calculate initial ground positions for drones"""
        return calculate_ground_formation(
            self.num_drones,
            self.world_size,
            FORMATION_SPACING
        )
    
    def _init_drones(self):
        """Initialize drone objects"""
        for i, pos in enumerate(self.ground_positions):
            drone = Drone(
                pos[0], pos[1], pos[2],  # Initial position
                pos[0], pos[1], pos[2],  # Target position
                i,                       # ID
                is_aerial_station=False
            )
            self.drones.append(drone)
    
    def _init_visualization(self):
        """Initialize visualization components"""
        self.fig = plt.figure(figsize=(16, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.view_angle = 0
        self.view_elevation = 30
    
    def set_formation_phase(self, phase):
        """Set the formation phase.
        
        Args:
            phase: Target formation phase
        """
        self.current_phase = phase
        if phase == FormationPhase.GROUND:
            self.target_positions = self._calculate_ground_formation()
        elif phase == FormationPhase.CUBE:
            self.target_positions = self._calculate_cube_formation()
        elif phase == FormationPhase.SPHERE:
            self.target_positions = self._calculate_sphere_formation()
        elif phase == FormationPhase.PYRAMID:
            self.target_positions = self._calculate_pyramid_formation()
        elif phase == FormationPhase.DNA:
            self.target_positions = self._calculate_dna_formation()
    
    def _calculate_ground_formation(self):
        """Calculate ground formation positions"""
        return calculate_ground_formation(
            self.num_drones,
            self.center_x,
            self.center_y
        )
    
    def _calculate_cube_formation(self):
        """Calculate positions for cube formation"""
        return calculate_cube_formation(
            self.num_drones,
            self.center_x,
            self.center_y,
            self.formation_spacing
        )
    
    def _calculate_sphere_formation(self):
        """Calculate sphere formation positions"""
        return calculate_sphere_formation(
            self.num_drones,
            self.center_x,
            self.center_y
        )
    
    def _calculate_pyramid_formation(self):
        """Calculate pyramid formation positions"""
        return calculate_pyramid_formation(
            self.num_drones,
            self.center_x,
            self.center_y
        )
    
    def _calculate_dna_formation(self):
        """Calculate DNA formation positions"""
        return calculate_dna_formation(
            self.num_drones,
            self.center_x,
            self.center_y
        )
    
    def update_formation(self):
        """Update formation state and positions"""
        # Update drone positions
        self.update_positions(self.drones, self.target_positions)
        
        # Check phase completion
        current_time = time.time()
        phase_duration = self.phase_durations[self.current_phase]
        
        if (current_time - self.phase_start_time >= phase_duration and
            self.check_convergence(self.drones)):
            self._advance_formation_phase()
            
    def check_convergence(self, drones):
        """Check if all drones have reached their target positions.
        
        Args:
            drones: List of drones to check
            
        Returns:
            bool: True if all drones have converged, False otherwise
        """
        for drone in drones:
            if not drone.target_x or not drone.target_y or not drone.target_z:
                continue
                
            distance = np.sqrt(
                (drone.x - drone.target_x) ** 2 +
                (drone.y - drone.target_y) ** 2 +
                (drone.z - drone.target_z) ** 2
            )
            if distance > CONVERGENCE_THRESHOLD:
                return False
        return True
    
    def _advance_formation_phase(self):
        """Advance to the next formation phase"""
        self.current_formation_index += 1
        if self.current_formation_index >= len(self.formation_sequence):
            self.current_formation_index = 0
            
        self.current_phase = self.formation_sequence[self.current_formation_index]
        self.phase_start_time = time.time()
        
        # Calculate new target positions based on phase
        self._update_target_positions()
    
    def _update_target_positions(self):
        """Update target positions based on current phase"""
        if self.current_phase == FormationPhase.GROUND:
            self.target_positions = self._calculate_ground_formation()
        elif self.current_phase == FormationPhase.CUBE:
            self.target_positions = self._calculate_cube_formation()
        elif self.current_phase == FormationPhase.SPHERE:
            self.target_positions = self._calculate_sphere_formation()
        elif self.current_phase == FormationPhase.PYRAMID:
            self.target_positions = self._calculate_pyramid_formation()
        elif self.current_phase == FormationPhase.DNA:
            self.target_positions = self._calculate_dna_formation()
        elif self.current_phase == FormationPhase.LANDING:
            self.target_positions = self.ground_positions
            
        # Update drone target positions
        for drone, target in zip(self.drones, self.target_positions):
            drone.target_x, drone.target_y, drone.target_z = target
    
    def get_target_positions(self):
        """Get current target positions.
        
        Returns:
            List of (x, y, z) tuples for target positions
        """
        return self.target_positions
