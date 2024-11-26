import numpy as np
import time

class BaseStation:
    """Base class for all station types"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.time_offset = 0
        self.last_sync_time = time.time()

    def get_position(self):
        """Return current position"""
        return (self.x, self.y, self.z)

class GroundStation(BaseStation):
    """Ground station serves as the time reference"""
    def __init__(self, x, y, z):
        super().__init__(x, y, z)
        self.time_offset = 0  # Reference time

    def get_reference_time(self):
        """Provide reference time"""
        return time.time()

class AerialBaseStation(BaseStation):
    """Aerial station with clock drift simulation"""
    def __init__(self, x, y, z, station_id):
        super().__init__(x, y, z)
        self.id = station_id
        self.clock_drift = np.random.normal(1e-6, 1e-7)  # Random clock drift
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def update_clock(self):
        """Simulate clock drift"""
        current_time = time.time()
        dt = current_time - self.last_sync_time
        self.time_offset += self.clock_drift * dt
        self.last_sync_time = current_time

    def set_target_position(self, x, y, z):
        """Set target position for movement"""
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def get_target_position(self):
        """Get target position"""
        return (self.target_x, self.target_y, self.target_z)
