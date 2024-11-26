"""
Positioning system module for drone swarm.
This module handles TDOA positioning and time synchronization management.
"""

import time
import numpy as np
from .time_sync import TimeSyncManager

class PositioningSystem:
    """System for managing positioning and time synchronization of drone swarm"""

    def __init__(self, ground_station, aerial_stations):
        """Initialize the positioning system

        Args:
            ground_station: Reference ground station
            aerial_stations: List of aerial base stations
        """
        self.ground_station = ground_station
        self.aerial_stations = aerial_stations
        self.time_sync = TimeSyncManager(ground_station, aerial_stations)
        self.last_sync = time.time()
        self.sync_interval = 0.1  # 100ms synchronization interval

    def update_time_sync(self):
        """Periodically update time synchronization

        Updates clock drift for all aerial stations and performs
        network-wide time synchronization if the sync interval has elapsed.
        """
        current_time = time.time()
        if current_time - self.last_sync >= self.sync_interval:
            # Update clock drift for all stations
            for station in self.aerial_stations:
                station.update_clock()
            # Perform network synchronization
            self.time_sync.synchronize_network()
            self.last_sync = current_time

    def tdoa_positioning(self, drone):
        """Perform TDOA positioning considering time synchronization

        This method implements Time Difference of Arrival (TDOA) positioning
        using the ground station as reference and considering time offsets
        of all stations.

        Args:
            drone: Target drone for positioning

        Returns:
            list: TDOA measurements relative to ground station
        """
        # Update time synchronization first
        self.update_time_sync()

        # Collect TDOA measurements
        measurements = []
        reference_time = self.ground_station.get_reference_time()

        # Calculate propagation time to reference station
        ref_dist = np.sqrt(
            (self.ground_station.x - drone.x)**2 +
            (self.ground_station.y - drone.y)**2 +
            (self.ground_station.z - drone.z)**2
        )
        ref_time = ref_dist / 299792458.0  # Speed of light propagation

        for station in self.aerial_stations:
            # Calculate propagation time to each aerial station
            dist = np.sqrt(
                (station.x - drone.x)**2 +
                (station.y - drone.y)**2 +
                (station.z - drone.z)**2
            )
            prop_time = dist / 299792458.0

            # Calculate TDOA (considering time offset)
            tdoa = (prop_time + station.time_offset) - (ref_time + self.ground_station.time_offset)

            # Add measurement noise
            tdoa += np.random.normal(0, 1e-10)  # 100 picosecond noise
            measurements.append(tdoa)

        return measurements
