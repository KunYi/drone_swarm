#!/usr/bin/env python3
"""
Main entry point for drone swarm formation control system.
Provides both simulation and testing capabilities.
"""

import argparse
import logging
import sys
from typing import Optional

from formations.formation_control import FormationControl
from formations.phase import FormationPhase

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def run_simulation(num_drones: int = 125, debug: bool = False):
    """Run the formation control simulation
    
    Args:
        num_drones: Number of drones to simulate
        debug: Enable debug logging if True
    """
    if debug:
        logger.setLevel(logging.DEBUG)
        
    logger.info(f"Starting simulation with {num_drones} drones")
    
    try:
        # Initialize formation control
        formation = FormationControl(num_drones)
        
        # Start animation
        formation.animate()
        
    except Exception as e:
        logger.error(f"Simulation failed: {str(e)}")
        raise

def test_formation(num_drones: int = 8, formation_type: Optional[FormationPhase] = None):
    """Test specific formation without animation
    
    Args:
        num_drones: Number of drones to test with
        formation_type: Specific formation to test, or None for all formations
    """
    logger.info(f"Starting formation test with {num_drones} drones")
    
    try:
        # Initialize formation control
        formation = FormationControl(num_drones)
        
        # Test specific formation if specified
        if formation_type:
            formations_to_test = [formation_type]
        else:
            formations_to_test = [
                FormationPhase.CUBE,
                FormationPhase.SPHERE,
                FormationPhase.PYRAMID,
                FormationPhase.DNA
            ]
            
        # Test each formation
        for phase in formations_to_test:
            logger.info(f"Testing {phase.name} formation")
            
            # Calculate formation positions
            positions = formation._calculate_formation_positions(phase)
            
            # Set formation targets
            formation.manager.set_formation_targets(positions)
            
            # Run simulation steps
            max_steps = 1000
            step = 0
            dt = 0.05  # 50ms time step
            
            while step < max_steps and not formation.manager.is_formation_complete():
                formation.manager.update_drone_positions(dt)
                step += 1
                
                if step % 20 == 0:  # Log progress every second
                    logger.debug(f"Step {step}: Formation progress...")
                    
            # Check results
            if formation.manager.is_formation_complete():
                logger.info(f"{phase.name} formation completed in {step} steps")
            else:
                logger.warning(f"{phase.name} formation did not converge in {max_steps} steps")
                
    except Exception as e:
        logger.error(f"Formation test failed: {str(e)}")
        raise

def main():
    """Main entry point with command line argument parsing"""
    parser = argparse.ArgumentParser(description="Drone Swarm Formation Control System")
    
    # Add arguments
    parser.add_argument(
        "--num-drones", 
        type=int, 
        default=127,
        help="Number of drones to simulate (default: 125)"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run formation tests instead of simulation"
    )
    parser.add_argument(
        "--formation",
        type=str,
        choices=["CUBE", "SPHERE", "PYRAMID", "DNA"],
        help="Test specific formation type"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    
    args = parser.parse_args()
    
    try:
        if args.test:
            formation_type = FormationPhase[args.formation] if args.formation else None
            test_formation(args.num_drones, formation_type)
        else:
            run_simulation(args.num_drones, args.debug)
            
    except Exception as e:
        logger.error(f"Program failed: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
