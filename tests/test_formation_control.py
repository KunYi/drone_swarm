"""Test cases for formation control system."""
import unittest
import numpy as np
from formations.controller import FormationController, DistributedController
from formations.phase import FormationPhase
from core.config import (
    SAFE_DISTANCE,
    CONVERGENCE_THRESHOLD,
    MAX_DRONES_PER_REGION
)

class TestFormationControl(unittest.TestCase):
    """Test cases for formation control."""
    
    def setUp(self):
        """Set up test cases."""
        self.num_drones = 27  # 3x3x3 cube
        self.controller = FormationController(self.num_drones)
        
    def test_initialization(self):
        """Test controller initialization."""
        self.assertEqual(len(self.controller.drones), self.num_drones)
        self.assertIsNotNone(self.controller.ground_station)
        self.assertTrue(len(self.controller.aerial_stations) > 0)
        
    def test_safe_distance(self):
        """Test minimum safe distance between drones."""
        self.controller.update_formation()
        positions = np.array([
            [d.x, d.y, d.z] for d in self.controller.drones
        ])
        
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                distance = np.linalg.norm(positions[i] - positions[j])
                self.assertGreaterEqual(
                    distance,
                    SAFE_DISTANCE,
                    f"Drones {i} and {j} are too close: {distance} < {SAFE_DISTANCE}"
                )
                
    def test_formation_transition(self):
        """Test formation phase transitions."""
        # Initial phase should be GROUND
        self.assertEqual(self.controller.current_phase, FormationPhase.GROUND)
        
        # Set target positions for all drones
        self.controller.set_formation_phase(FormationPhase.CUBE)
        
        # Update positions to match targets
        for drone in self.controller.drones:
            if drone.target_x is not None:
                drone.x = drone.target_x
                drone.y = drone.target_y
                drone.z = drone.target_z
        
        # Check convergence
        self.assertTrue(
            self.controller.check_convergence(self.controller.drones),
            "Not all drones reached their target positions"
        )

class TestDistributedControl(unittest.TestCase):
    """Test cases for distributed formation control."""
    
    def setUp(self):
        """Set up test cases."""
        self.num_drones = 125
        self.controller = DistributedController(self.num_drones)
        
    def test_region_assignment(self):
        """Test drone region assignment."""
        # Get a region
        region = next(iter(self.controller.regions.values()))
        
        # Clear all drone assignments
        for r in self.controller.regions.values():
            r.drones.clear()
        
        # Move all drones out of bounds first
        for drone in self.controller.drones:
            drone.x = -1
            drone.y = -1
            drone.z = -1
        
        # Add a single drone to the region
        drone = self.controller.drones[0]
        drone.x = region.x + region.size / 2
        drone.y = region.y + region.size / 2
        drone.z = 0
        
        # Update assignments
        self.controller._assign_drones_to_regions()
        
        # Check assignment
        self.assertEqual(
            len(region.drones),
            1,
            "Region should have exactly one drone"
        )
        self.assertEqual(
            region.drones[0],
            drone,
            "Region should contain the assigned drone"
        )
        
    def test_region_communication(self):
        """Test inter-region communication."""
        # Get a region
        region = next(iter(self.controller.regions.values()))
        
        # Get neighbors
        neighbors = region.get_neighbor_regions()
        
        # Check if neighbors are valid
        for neighbor in neighbors:
            # Check if neighbor is within valid distance
            dx = abs(region.x - neighbor.x)
            dy = abs(region.y - neighbor.y)
            self.assertLessEqual(
                dx,
                region.size,
                "Neighbor region too far in x direction"
            )
            self.assertLessEqual(
                dy,
                region.size,
                "Neighbor region too far in y direction"
            )

if __name__ == '__main__':
    unittest.main()
