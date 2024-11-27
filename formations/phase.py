"""Formation phase definitions for the drone swarm system."""
from enum import Enum, auto

class FormationPhase(Enum):
    """Enum representing different phases of drone formation."""
    PREPARE = auto()
    STATIONS_TAKEOFF = auto()
    GROUND = auto()
    CUBE = auto()
    SPHERE = auto()
    PYRAMID = auto()
    DNA = auto()
    LANDING = auto()
    STATIONS_LANDING = auto()
    EXIT = auto()
