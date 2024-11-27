"""
Main formation control module that integrates manager, visualizer and state management.
"""

import time
import numpy as np
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib import animation

from formations.manager import FormationManager
from formations.visualizer import FormationVisualizer
from formations.state_manager import FormationStateManager
from formations.phase import FormationPhase
from formations import (
    calculate_cube_formation,
    calculate_sphere_formation,
    calculate_pyramid_formation,
    calculate_dna_formation
)

class FormationControl:
    """Main formation control class that integrates all components"""

    def __init__(self, num_drones=125):
        """Initialize formation control system

        Args:
            num_drones: Number of drones to control (default: 125)
        """
        # Initialize components
        self.manager = FormationManager(num_drones)
        self.visualizer = FormationVisualizer(self.manager.world_size)
        self.state_manager = FormationStateManager()

        # Animation control
        self._animation = None
        self._is_running = False

    def _calculate_formation_positions(self, formation_type: FormationPhase) -> List[Tuple[float, float, float]]:
        """Calculate target positions for current formation

        Args:
            formation_type: Type of formation to calculate

        Returns:
            List of (x, y, z) positions for the formation
        """
        if formation_type in [
            FormationPhase.PREPARE,
            FormationPhase.STATIONS_TAKEOFF,
            FormationPhase.STATIONS_LANDING,
            FormationPhase.EXIT,
            FormationPhase.GROUND
        ]:
            return self.manager.ground_positions

        center_x = self.manager.center_x
        center_y = self.manager.center_y
        spacing = 2.0

        try:
            if formation_type == FormationPhase.CUBE:
                return calculate_cube_formation(self.manager.num_drones, center_x, center_y, spacing)
            elif formation_type == FormationPhase.SPHERE:
                return calculate_sphere_formation(self.manager.num_drones, center_x, center_y, spacing)
            elif formation_type == FormationPhase.PYRAMID:
                return calculate_pyramid_formation(self.manager.num_drones, center_x, center_y)
            elif formation_type == FormationPhase.DNA:
                return calculate_dna_formation(self.manager.num_drones, center_x, center_y, spacing)
            elif formation_type == FormationPhase.LANDING:
                return self._calculate_landing_positions()
        except Exception as e:
            print(f"Error calculating {formation_type} formation positions: {str(e)}")
            return self.manager.ground_positions

        return self.manager.ground_positions

    def _calculate_landing_positions(self) -> List[Tuple[float, float, float]]:
        """Calculate intermediate positions for landing phase"""
        phase_progress = self.state_manager.get_phase_progress()
        current_positions = self.manager.get_drone_positions()
        landing_positions = []

        for i, (current_pos, ground_pos) in enumerate(zip(current_positions, self.manager.ground_positions)):
            # Use cosine function for smooth landing
            progress = 0.5 * (1 - np.cos(phase_progress * np.pi))

            if progress < 0.7:  # First 70% of time for height reduction
                x = current_pos[0]
                y = current_pos[1]
                z = current_pos[2] * (1 - progress/0.7)
            else:  # Remaining time for horizontal movement
                horizontal_progress = (progress - 0.7) / 0.3
                x = current_pos[0] + (ground_pos[0] - current_pos[0]) * horizontal_progress
                y = current_pos[1] + (ground_pos[1] - current_pos[1]) * horizontal_progress
                z = 0

            landing_positions.append((x, y, z))

        return landing_positions

    def _update_frame(self, frame: int) -> None:
        """Update animation frame

        Args:
            frame: Frame number
        """
        if not self._is_running:
            return

        # Calculate time step
        current_time = time.time()
        dt = current_time - self._last_frame_time
        self._last_frame_time = current_time

        # Update formation phase
        if self.state_manager.update_phase():
            # Phase changed, calculate new target positions
            new_phase = self.state_manager.get_current_phase()
            self.manager.target_positions = self._calculate_formation_positions(new_phase)

        # Update drone positions
        self.manager.update_drone_positions(dt)

        # Update visualization
        self.visualizer.update_view(dt)
        self.visualizer.plot_drones(
            self.manager.get_drone_positions(),
            self.manager.get_target_positions(),
            (self.manager.ground_station.x, self.manager.ground_station.y, self.manager.ground_station.z),
            [(station.x, station.y, station.z) for station in self.manager.aerial_stations]
        )

        # Check if formation sequence is complete
        if self.state_manager.is_complete():
            self.stop_animation()

    def animate(self):
        """Start formation animation"""
        self._is_running = True
        self._last_frame_time = time.time()

        # Set up animation
        self._animation = animation.FuncAnimation(
            self.visualizer.fig,
            self._update_frame,
            interval=50,  # 20 FPS
            blit=False
        )

        # Show the animation
        plt.show()

    def stop_animation(self):
        """Stop the animation"""
        self._is_running = False
        if self._animation is not None:
            self._animation.event_source.stop()

    def reset(self):
        """Reset the formation control to initial state"""
        self.state_manager.reset()
        self.manager.target_positions = self.manager.ground_positions.copy()
        for drone in self.manager.drones:
            pos = self.manager.ground_positions[drone.id]
            drone.x, drone.y, drone.z = pos
            drone.target_x, drone.target_y, drone.target_z = pos
