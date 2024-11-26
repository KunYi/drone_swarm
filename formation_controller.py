"""
Formation controller module for drone swarm.
This module handles all formation-related calculations and transitions.
"""

import numpy as np
import time
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from formation_phase import FormationPhase
from formations import (
    calculate_ground_formation,
    calculate_cube_formation,
    calculate_sphere_formation,
    calculate_pyramid_formation,
    calculate_dna_formation
)
from safety.collision_avoidance import CollisionAvoidance
from planning.path_planner import PathPlanner

class FormationControl:
    def __init__(self, num_drones=125):
        self.world_size = 40
        self.center_x = self.world_size / 2
        self.center_y = self.world_size / 2
        self.spacing = 2.0
        self.num_drones = num_drones

        # Initialize multiprocessing pool
        self.pool = mp.Pool()
        self.thread_pool = ThreadPoolExecutor(max_workers=4)

        # Formation parameters
        self.formation_positions = []
        self.current_positions = []
        self.target_positions = []

        # Initialize formation phases and durations
        self.formation_phases = {
            FormationPhase.PREPARE: 2.0,
            FormationPhase.STATIONS_TAKEOFF: 5.0,
            FormationPhase.GROUND: 5.0,
            FormationPhase.CUBE: 10.0,
            FormationPhase.SPHERE: 10.0,
            FormationPhase.PYRAMID: 10.0,
            FormationPhase.DNA: 10.0,
            FormationPhase.LANDING: 5.0,
            FormationPhase.STATIONS_LANDING: 5.0,
            FormationPhase.EXIT: 2.0
        }

        # Initialize helper classes
        self.collision_avoidance = CollisionAvoidance(
            min_distance=1.5,
            max_speed=0.5,
            boundary_margin=2.0,
            world_size=self.world_size
        )
        self.path_planner = PathPlanner(max_speed=0.5)

    def calculate_formation_positions(self, formation_type):
        """Calculate positions for the specified formation type"""
        center_x = self.center_x
        center_y = self.center_y

        if formation_type == FormationPhase.GROUND:
            positions = calculate_ground_formation(
                self.num_drones, center_x, center_y, self.spacing)

        elif formation_type == FormationPhase.CUBE:
            positions = calculate_cube_formation(
                self.num_drones, center_x, center_y, self.spacing)

        elif formation_type == FormationPhase.SPHERE:
            positions = calculate_sphere_formation(
                self.num_drones, center_x, center_y, self.spacing)

        elif formation_type == FormationPhase.PYRAMID:
            positions = calculate_pyramid_formation(
                self.num_drones, center_x, center_y)

        elif formation_type == FormationPhase.DNA:
            positions = calculate_dna_formation(
                self.num_drones, center_x, center_y, self.spacing)

        else:
            # Default to hovering formation
            positions = [(center_x, center_y, 3.0)] * self.num_drones

        return positions[:self.num_drones]

    def optimize_assignments(self, current_positions, target_positions):
        """Optimize drone assignments to minimize total distance"""
        return self.path_planner.optimize_assignments(current_positions, target_positions)

    def calculate_step_positions(self, current_positions, dt):
        """Calculate next step positions for all drones"""
        # Calculate next positions using path planner
        new_positions = []
        for current_pos, target_pos in zip(current_positions, self.target_positions):
            new_pos = self.path_planner.calculate_step(current_pos, target_pos, dt)
            new_positions.append(new_pos)

        # Apply collision avoidance
        new_positions = self.collision_avoidance.avoid_collisions(new_positions, dt)

        return new_positions

    def get_formation_duration(self, formation_type):
        """Get the duration for a specific formation phase"""
        return self.formation_phases.get(formation_type, 5.0)  # Default to 5 seconds
