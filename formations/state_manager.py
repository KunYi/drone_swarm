"""
Formation state manager module for managing formation states and transitions.
"""

import time
from typing import Dict, List
from formations.phase import FormationPhase

class FormationStateManager:
    """Manager class for formation states and transitions"""

    def __init__(self):
        self.formation_sequence = [
            FormationPhase.PREPARE,          # Initial state display
            FormationPhase.STATIONS_TAKEOFF, # Aerial stations takeoff
            FormationPhase.GROUND,          # Ground formation
            FormationPhase.CUBE,            # Cube formation
            FormationPhase.SPHERE,          # Sphere formation
            FormationPhase.PYRAMID,         # Pyramid formation
            FormationPhase.DNA,            # DNA formation
            FormationPhase.LANDING,         # Drones landing
            FormationPhase.STATIONS_LANDING, # Aerial stations landing
            FormationPhase.EXIT            # Final state confirmation
        ]

        self.formation_duration = {
            FormationPhase.PREPARE: 3,          # 3 seconds to show initial state
            FormationPhase.STATIONS_TAKEOFF: 8,  # Time for aerial stations to take off
            FormationPhase.GROUND: 4,           # Ground formation time
            FormationPhase.CUBE: 15,            # Cube formation time
            FormationPhase.SPHERE: 12,          # Sphere formation time
            FormationPhase.PYRAMID: 22,         # Pyramid formation time
            FormationPhase.DNA: 16,            # DNA formation time
            FormationPhase.LANDING: 20,         # Drones landing time
            FormationPhase.STATIONS_LANDING: 12, # Aerial stations landing time
            FormationPhase.EXIT: 3             # 3 seconds to confirm end state
        }

        self.current_formation_index = 0
        self.current_phase = self.formation_sequence[0]
        self.start_time = time.time()
        self.phase_start_time = self.start_time
        self.last_progress_time = time.time()

    def update_phase(self) -> bool:
        """Update formation phase based on time

        Returns:
            bool: True if phase changed, False otherwise
        """
        current_time = time.time()
        phase_duration = self.formation_duration[self.current_phase]

        if current_time - self.phase_start_time >= phase_duration:
            self.current_formation_index += 1
            if self.current_formation_index >= len(self.formation_sequence):
                return False

            self.current_phase = self.formation_sequence[self.current_formation_index]
            self.phase_start_time = current_time
            return True

        return False

    def get_phase_progress(self) -> float:
        """Get progress of current phase (0.0 to 1.0)"""
        current_time = time.time()
        phase_duration = self.formation_duration[self.current_phase]
        progress = (current_time - self.phase_start_time) / phase_duration
        return min(1.0, progress)

    def get_current_phase(self) -> FormationPhase:
        """Get current formation phase"""
        return self.current_phase

    def is_complete(self) -> bool:
        """Check if formation sequence is complete"""
        return self.current_formation_index >= len(self.formation_sequence)

    def reset(self):
        """Reset state manager to initial state"""
        self.current_formation_index = 0
        self.current_phase = self.formation_sequence[0]
        self.start_time = time.time()
        self.phase_start_time = self.start_time
        self.last_progress_time = time.time()
