import numpy as np
import time

class UWBTimeSync:
    def __init__(self):
        self.clock_drift = 1e-6  # 1 PPM時鐘漂移
        self.last_sync = time.time()
        self.time_offset = 0
        self.sync_interval = 0.1  # 100ms同步間隔

    def two_way_ranging(self, initiator, responder):
        """Simulate TWR process"""
        # T1: Send time
        t1 = time.time() + initiator.time_offset

        # Simulate propagation delay
        distance = np.sqrt(
            (initiator.x - responder.x)**2 +
            (initiator.y - responder.y)**2 +
            (initiator.z - responder.z)**2
        )
        propagation_delay = distance / 299792458.0  # Speed of light propagation

        # T2: Receive time
        t2 = t1 + propagation_delay + responder.time_offset

        # T3: Response time
        t3 = t2 + 0.000001  # 1 microsecond processing delay

        # T4: Final receive time
        t4 = t3 + propagation_delay + initiator.time_offset

        # Calculate time difference
        round_trip_time = (t4 - t1) - (t3 - t2)
        one_way_time = round_trip_time / 2

        # Add measurement noise
        noise = np.random.normal(0, 1e-10)  # 100 picosecond noise
        return one_way_time + noise


class TimeSyncManager:
    def __init__(self, ground_station, aerial_stations):
        self.ground_station = ground_station
        self.aerial_stations = aerial_stations
        self.uwb_sync = UWBTimeSync()
        self.sync_history = []

    def synchronize_network(self):
        """Perform network time synchronization"""
        sync_record = {
            'timestamp': time.time(),
            'stations': []
        }

        # First phase: Synchronize with ground station
        for station in self.aerial_stations:
            # Perform multiple TWRs to improve accuracy
            time_offsets = []
            for _ in range(5):
                twr_time = self.uwb_sync.two_way_ranging(
                    station, self.ground_station)
                time_offsets.append(twr_time)

            # Use median to filter out outliers
            station.time_offset = np.median(time_offsets)
            sync_record['stations'].append({
                'station_id': station.id,
                'time_offset': station.time_offset,
                'measurements': time_offsets
            })

        # Second phase: Synchronize aerial stations with each other
        for i, station1 in enumerate(self.aerial_stations):
            for j, station2 in enumerate(self.aerial_stations[i+1:]):
                twr_time = self.uwb_sync.two_way_ranging(station1, station2)
                # Adjust relative time offset
                mean_offset = (station1.time_offset + station2.time_offset) / 2
                station1.time_offset = station2.time_offset = mean_offset

                sync_record['stations'].append({
                    'station_pair': [station1.id, station2.id],
                    'final_offset': mean_offset,
                    'twr_time': twr_time
                })

        # Add sync record to history
        self.sync_history.append(sync_record)
