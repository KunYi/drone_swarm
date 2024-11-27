"""
Test suite for drone formation control system.
"""

import unittest
import numpy as np
from typing import List, Tuple

from formations.formation_control import FormationControl
from formations.phase import FormationPhase
from core.config import SAFE_DISTANCE, CONVERGENCE_THRESHOLD

class TestFormationControl(unittest.TestCase):
    """Test cases for formation control system"""
    
    def setUp(self):
        """Set up test cases"""
        self.num_drones = 8  # Use small number for testing
        self.formation = FormationControl(self.num_drones)
        
    def test_initialization(self):
        """Test proper initialization of formation control"""
        self.assertEqual(len(self.formation.manager.drones), self.num_drones)
        self.assertEqual(len(self.formation.manager.ground_positions), self.num_drones)
        self.assertEqual(len(self.formation.manager.target_positions), self.num_drones)
        
    def test_ground_formation_spacing(self):
        """Test that ground formation maintains safe distances"""
        positions = self.formation.manager.ground_positions
        
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                pos1 = positions[i]
                pos2 = positions[j]
                distance = np.sqrt(
                    (pos1[0] - pos2[0])**2 +
                    (pos1[1] - pos2[1])**2 +
                    (pos1[2] - pos2[2])**2
                )
                self.assertGreaterEqual(distance, SAFE_DISTANCE)
                
    def test_cube_formation(self):
        """Test cube formation calculation and convergence"""
        # Calculate cube formation positions
        positions = self.formation._calculate_formation_positions(FormationPhase.CUBE)
        self.formation.manager.set_formation_targets(positions)
        
        # Run simulation steps
        max_steps = 1000
        step = 0
        dt = 0.05
        
        while step < max_steps and not self.formation.manager.is_formation_complete():
            self.formation.manager.update_drone_positions(dt)
            step += 1
            
        self.assertTrue(
            self.formation.manager.is_formation_complete(),
            "Cube formation did not converge"
        )
        
    def test_collision_avoidance(self):
        """Test that drones maintain safe distances during movement"""
        # Set up a potential collision scenario
        self.formation.manager.drones[0].x = 10
        self.formation.manager.drones[0].y = 10
        self.formation.manager.drones[0].z = 10
        
        self.formation.manager.drones[1].x = 10 + SAFE_DISTANCE * 0.5
        self.formation.manager.drones[1].y = 10
        self.formation.manager.drones[1].z = 10
        
        # Run a few update steps
        for _ in range(10):
            self.formation.manager.update_drone_positions(0.05)
            
            # Check distances between all drones
            positions = self.formation.manager.get_drone_positions()
            for i in range(len(positions)):
                for j in range(i + 1, len(positions)):
                    pos1 = positions[i]
                    pos2 = positions[j]
                    distance = np.sqrt(
                        (pos1[0] - pos2[0])**2 +
                        (pos1[1] - pos2[1])**2 +
                        (pos1[2] - pos2[2])**2
                    )
                    self.assertGreaterEqual(
                        distance,
                        SAFE_DISTANCE * 0.9,  # Allow small tolerance
                        f"Collision detected between drones {i} and {j}"
                    )
                    
    def test_formation_completion_detection(self):
        """Test that formation completion is correctly detected"""
        # Set all drones to their targets
        for i, drone in enumerate(self.formation.manager.drones):
            target = self.formation.manager.target_positions[i]
            drone.x = target[0]
            drone.y = target[1]
            drone.z = target[2]
            
        self.assertTrue(
            self.formation.manager.is_formation_complete(),
            "Formation completion not detected"
        )
        
        # Move one drone slightly
        self.formation.manager.drones[0].x += CONVERGENCE_THRESHOLD * 2
        self.assertFalse(
            self.formation.manager.is_formation_complete(),
            "Incomplete formation incorrectly marked as complete"
        )
        
if __name__ == '__main__':
    unittest.main()
