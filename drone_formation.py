#!/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.optimize import linear_sum_assignment
from multiprocessing import Pool
import time
import multiprocessing as mp
from functools import partial
from itertools import product
from sklearn.cluster import KMeans
from concurrent.futures import ThreadPoolExecutor
from positioning import AerialBaseStation, GroundStation, UWBTimeSync, TimeSyncManager, PositioningSystem
from formation_controller import FormationControl
from formation_phase import FormationPhase
from spatial import SpatialGrid
from core import Drone
from formations import (
    calculate_cube_formation,
    calculate_sphere_formation,
    calculate_pyramid_formation,
    calculate_dna_formation
)

class FormationControl:
    def __init__(self, num_drones=125):  # Default to 125 drones (5x5x5)
        # Expand coordinate system
        self.world_size = 40  # Expand world size to accommodate more drones
        self.center_x = self.world_size / 2
        self.center_y = self.world_size / 2

        # Create ground station, placed at the edge of the field
        self.ground_station = GroundStation(self.world_size * 0.8, self.world_size * 0.8, 0)

        # Create more aerial stations, distributed at different locations
        self.aerial_stations = [
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.2, 0, 0),  # Front left
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.2, 0, 1),  # Front right
            AerialBaseStation(self.world_size * 0.2, self.world_size * 0.8, 0, 2),  # Back left
            AerialBaseStation(self.world_size * 0.5, self.world_size * 0.5, 0, 3),  # Center
            AerialBaseStation(self.world_size * 0.8, self.world_size * 0.8, 0, 4),  # Back right
        ]

        self.num_drones = num_drones
        self.positioning_system = PositioningSystem(
            self.ground_station, self.aerial_stations)

        # Increase process pool size for better performance
        self.pool = Pool(processes=mp.cpu_count())

        # Initialize drones
        spacing = 1.2  # Increase initial spacing
        self.ground_positions = []
        rows = int(np.ceil(np.sqrt(num_drones)))
        cols = int(np.ceil(num_drones / rows))

        for i in range(rows):
            for j in range(cols):
                if len(self.ground_positions) < num_drones:
                    x = self.world_size * 0.2 + i * spacing
                    y = self.world_size * 0.2 + j * spacing
                    self.ground_positions.append((x, y, 0))

        # Create drone objects
        self.drones = []
        for i, pos in enumerate(self.ground_positions):
            drone = Drone(pos[0], pos[1], pos[2],  # Initial position
                        pos[0], pos[1], pos[2],    # Target position (initially the same as current position)
                        i,                         # ID number
                        is_aerial_station=False)   # Not an aerial station
            self.drones.append(drone)

        # Initialize target positions to ground positions
        self.target_positions = self.ground_positions.copy()

        # Initialize animation-related attributes
        self.fig = plt.figure(figsize=(16, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.view_angle = 0
        self.view_elevation = 30

        # Set formation sequence and duration
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
        self.current_formation_index = 0
        self.current_phase = self.formation_sequence[0]
        self.formation_duration = {
            FormationPhase.PREPARE: 3,          # 3 seconds to show initial state
            FormationPhase.STATIONS_TAKEOFF: 8,  # Time for aerial stations to take off
            FormationPhase.GROUND: 6,           # Ground formation time
            FormationPhase.CUBE: 14,            # Cube formation time
            FormationPhase.SPHERE: 15,          # Sphere formation time
            FormationPhase.PYRAMID: 22,         # Pyramid formation time
            FormationPhase.DNA: 16,            # DNA formation time
            FormationPhase.LANDING: 20,         # Drones landing time
            FormationPhase.STATIONS_LANDING: 12, # Aerial stations landing time
            FormationPhase.EXIT: 3             # 3 seconds to confirm end state
        }

        # Adjust convergence parameters
        self.convergence_threshold = 0.15  # Slightly relax convergence threshold to accommodate more drones

        # Initialize time-related variables
        self.start_time = time.time()
        self.phase_start_time = self.start_time
        self.last_progress_time = time.time()

        # Initialize spatial grid
        self.spatial_grid = SpatialGrid(self.world_size)

    def _calculate_formation_positions(self, formation_type):
        """Calculate basic formation positions"""
        positions = []
        center_x = self.center_x
        center_y = self.center_y

        if formation_type in [FormationPhase.PREPARE, FormationPhase.STATIONS_TAKEOFF, FormationPhase.STATIONS_LANDING, FormationPhase.EXIT, FormationPhase.GROUND]:
            return self.ground_positions

        elif formation_type == FormationPhase.CUBE:
            spacing = 2.0  # Fixed spacing for cube formation
            try:
                positions = calculate_cube_formation(self.num_drones, center_x, center_y, spacing)
            except Exception as e:
                print(f"Error calculating cube formation positions: {str(e)}")
                return self.ground_positions

        elif formation_type == FormationPhase.SPHERE:
            spacing = 2.0  # Fixed spacing for sphere formation
            try:
                positions = calculate_sphere_formation(self.num_drones, center_x, center_y, spacing)
            except Exception as e:
                print(f"Error calculating sphere formation positions: {str(e)}")
                return self.ground_positions

        elif formation_type == FormationPhase.PYRAMID:
            try:
                positions = calculate_pyramid_formation(self.num_drones, center_x, center_y)
            except Exception as e:
                print(f"Error calculating pyramid formation positions: {str(e)}")
                return self.ground_positions

        elif formation_type == FormationPhase.DNA:
            spacing = 2.0  # Fixed spacing for DNA formation
            try:
                positions = calculate_dna_formation(self.num_drones, center_x, center_y, spacing)
            except Exception as e:
                print(f"Error calculating DNA formation positions: {str(e)}")
                return self.ground_positions

        elif formation_type == FormationPhase.LANDING:
            return self.ground_positions

        # Ensure enough positions are returned
        if len(positions) < self.num_drones:
            print(f"Warning: {formation_type} formation returned {len(positions)} positions, but {self.num_drones} are needed")
            # Pad with ground positions if necessary
            positions.extend(self.ground_positions[len(positions):self.num_drones])
        elif len(positions) > self.num_drones:
            positions = positions[:self.num_drones]

        return positions

    def _optimize_target_assignment(self, current_positions, target_positions):
        """使用匈牙利算法優化目標點分配

        Args:
            current_positions: 當前無人機位置列表 [(x, y, z), ...]
            target_positions: 目標位置列表 [(x, y, z), ...]

        Returns:
            list: 優化後的目標位置列表
        """
        # 構建成本矩陣
        cost_matrix = np.zeros((len(current_positions), len(target_positions)))
        for i, current_pos in enumerate(current_positions):
            for j, target_pos in enumerate(target_positions):
                # 計算歐幾里得距離作為成本
                dx = current_pos[0] - target_pos[0]
                dy = current_pos[1] - target_pos[1]
                dz = current_pos[2] - target_pos[2]
                cost_matrix[i, j] = np.sqrt(dx*dx + dy*dy + dz*dz)

        # 使用匈牙利算法求解最優分配
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 根據分配結果重排目標位置
        optimized_targets = [None] * len(target_positions)
        for i, j in zip(row_ind, col_ind):
            optimized_targets[i] = target_positions[j]

        return optimized_targets

    def update_formation(self, frame):
        try:
            current_time = time.time()
            phase_elapsed_time = current_time - self.phase_start_time

            # Update positioning system
            self.positioning_system.update_time_sync()

            # Handle aerial stations movement
            if self.current_phase == FormationPhase.STATIONS_TAKEOFF:
                # Set target heights if not set
                if not hasattr(self, 'target_heights_set'):
                    self.target_heights = [8, 8, 8, 15, 8]  # Different heights for each station
                    for station, height in zip(self.aerial_stations, self.target_heights):
                        station.target_x = station.x  # Keep x,y position
                        station.target_y = station.y
                        station.target_z = height     # Set target height
                    self.target_heights_set = True

                # Move stations
                for station in self.aerial_stations:
                    dx = station.target_x - station.x
                    dy = station.target_y - station.y
                    dz = station.target_z - station.z

                    # Calculate distance to target
                    distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                    if distance > 0.1:
                        # Move stations gradually to their target positions
                        speed_factor = 0.05  # Reduced speed for smoother movement
                        station.x += dx * speed_factor
                        station.y += dy * speed_factor
                        station.z += dz * speed_factor

            elif self.current_phase == FormationPhase.STATIONS_LANDING:
                # Set landing targets if not set
                if not hasattr(self, 'landing_targets_set'):
                    for station in self.aerial_stations:
                        station.target_x = station.x  # Keep x,y position
                        station.target_y = station.y
                        station.target_z = 0          # Set target height to ground
                    self.landing_targets_set = True

                # Move stations
                for station in self.aerial_stations:
                    dx = station.target_x - station.x
                    dy = station.target_y - station.y
                    dz = station.target_z - station.z

                    # Calculate distance to target
                    distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                    if distance > 0.1:
                        # Move stations gradually to their target positions
                        speed_factor = 0.05  # Reduced speed for smoother movement
                        station.x += dx * speed_factor
                        station.y += dy * speed_factor
                        station.z += dz * speed_factor

            # Calculate target positions for drones
            target_positions = self._calculate_formation_positions(self.current_phase)

            # 獲取當前所有無人機的位置
            current_positions = [(drone.x, drone.y, drone.z) for drone in self.drones]

            # 使用匈牙利算法優化目標分配
            optimized_targets = self._optimize_target_assignment(current_positions, target_positions)

            # Update drone positions
            for i, drone in enumerate(self.drones):
                if i < len(optimized_targets):
                    drone.target_x, drone.target_y, drone.target_z = optimized_targets[i]
                    # 使用空間網格獲取鄰近無人機
                    nearby_drones = self.spatial_grid.get_nearby_drones(drone)
                    drone.update_position(nearby_drones)

            # Update the scatter plot
            positions = []
            colors = []

            # Add ground station
            positions.append((self.ground_station.x, self.ground_station.y, self.ground_station.z))
            colors.append('red')  # Ground station in red

            # Add aerial stations
            for station in self.aerial_stations:
                positions.append((station.x, station.y, station.z))
                colors.append('green')  # Aerial stations in green

            # Add drones
            for drone in self.drones:
                positions.append((drone.x, drone.y, drone.z))
                colors.append('blue')  # Drones in blue

            # Convert to numpy arrays for plotting
            positions = np.array(positions)
            x = positions[:, 0]
            y = positions[:, 1]
            z = positions[:, 2]

            if hasattr(self, 'scatter'):
                self.scatter.remove()
            self.scatter = self.ax.scatter(x, y, z, c=colors, marker='o')

            # Set axis limits
            self.ax.set_xlim([0, self.world_size])
            self.ax.set_ylim([0, self.world_size])

            # Only fix Z axis during aerial station related phases
            if self.current_phase in [FormationPhase.STATIONS_TAKEOFF, FormationPhase.STATIONS_LANDING]:
                self.ax.set_zlim([0, self.world_size * 0.6])
            else:
                # Calculate dynamic Z limit based on drone positions
                max_z = max(z) if len(z) > 0 else self.world_size * 0.6
                self.ax.set_zlim([0, max(max_z + 2, self.world_size * 0.3)])  # Add some padding above highest point

            # Update view angle
            self.update_view()

            # Check if formation change is needed
            if self.should_change_formation(phase_elapsed_time):
                old_phase = self.current_phase
                self.current_formation_index = (self.current_formation_index + 1) % len(self.formation_sequence)
                self.current_phase = self.formation_sequence[self.current_formation_index]
                self.phase_start_time = current_time

                # Reset phase-specific flags
                if hasattr(self, 'target_heights_set'):
                    delattr(self, 'target_heights_set')
                if hasattr(self, 'landing_targets_set'):
                    delattr(self, 'landing_targets_set')

                print(f"\nCompleted {old_phase}, transitioning to {self.current_phase}...")

            return (self.scatter,)

        except Exception as e:
            print(f"Error updating formation: {str(e)}")
            return (self.scatter,)

    def should_change_formation(self, elapsed_time):
        """Check if formation should be changed"""
        # Get current formation duration
        duration = self.formation_duration[self.current_phase]

        # Calculate max distance to target for all drones
        max_distance = 0
        for drone in self.drones:
            distance = drone.distance_to_target()
            max_distance = max(max_distance, distance)

        # For aerial station phases, check station positions
        if self.current_phase in [FormationPhase.STATIONS_TAKEOFF, FormationPhase.STATIONS_LANDING]:
            for station in self.aerial_stations:
                distance = np.sqrt((station.x - station.target_x)**2 +
                                 (station.y - station.target_y)**2 +
                                 (station.z - station.target_z)**2)
                max_distance = max(max_distance, distance)

        # Print progress every second
        current_time = time.time()
        if current_time - self.last_progress_time >= 1.0:
            print(f"Current Phase: {self.current_phase}, Max Distance to Target: {max_distance:.3f}, Phase Time: {elapsed_time:.1f}s")
            self.last_progress_time = current_time

        # Check if we should change formation
        if elapsed_time >= duration and max_distance < self.convergence_threshold:
            # Don't change if we're at the last phase
            if self.current_phase == FormationPhase.EXIT:
                return False
            return True

        return False

    def update_view(self):
        """Update view angle"""
        # Adjust view angle based on current phase
        if self.current_phase == FormationPhase.PREPARE:
            target_elevation = 45  # Prepare phase uses a higher view angle
            self.view_angle += 0.5
        elif self.current_phase == FormationPhase.STATIONS_TAKEOFF:
            target_elevation = 30  # Station takeoff phase requires a good 3D view
            self.view_angle += 0.8
        elif self.current_phase == FormationPhase.GROUND:
            target_elevation = 25  # Ground formation requires a lower view angle to show 3D effect
            self.view_angle += 1.0
        elif self.current_phase == FormationPhase.CUBE:
            target_elevation = 30  # Cube formation requires a good 3D view
            self.view_angle += 0.8
        elif self.current_phase == FormationPhase.SPHERE:
            target_elevation = 25  # Spherical formation requires a lower view angle to show 3D effect
            self.view_angle += 1.0
        elif self.current_phase == FormationPhase.PYRAMID:
            target_elevation = 35  # Pyramid formation requires a higher view angle to show layers
            self.view_angle += 0.6
        elif self.current_phase == FormationPhase.DNA:
            target_elevation = 20  # DNA spiral requires a lower view angle to show spiral effect
            self.view_angle += 1.2
        elif self.current_phase == FormationPhase.LANDING:
            target_elevation = 45  # Landing phase uses a higher view angle
            self.view_angle += 0.5
        elif self.current_phase == FormationPhase.STATIONS_LANDING:
            target_elevation = 30  # Station landing phase requires a good 3D view
            self.view_angle += 0.8
        elif self.current_phase == FormationPhase.EXIT:
            target_elevation = 45  # Exit phase uses a higher view angle
            self.view_angle += 0.5

        # Smoothly transition to target elevation
        self.view_elevation += (target_elevation - self.view_elevation) * 0.05

        # Update view angle
        self.ax.view_init(elev=self.view_elevation, azim=self.view_angle)

    def animate(self):
        """Create and save animation"""
        try:
            print("Starting animation...")

            # Calculate total frames needed
            total_duration = sum(self.formation_duration.values())
            fps = 24  # Increase FPS for smoother animation
            total_frames = int(total_duration * fps)

            # Initialize animation with increased frames
            anim = animation.FuncAnimation(
                self.fig,
                self.update_formation,
                frames=total_frames,  # Dynamic frame count based on total duration
                interval=1000/fps,    # Interval in milliseconds
                blit=True
            )

            # Configure writer with higher FPS
            writer = animation.FFMpegWriter(
                fps=fps,
                metadata=dict(artist='Drone Formation Simulator'),
                bitrate=2000
            )

            print(f"Saving animation... (Expected duration: {total_duration:.1f}s)")
            # Save animation
            anim.save('drone_formation.mp4', writer=writer)
            print("Animation successfully saved as 'drone_formation.mp4'")

        except Exception as e:
            print(f"Error in animation: {str(e)}")
        finally:
            print("Closing process pool...")
            self.pool.close()
            print("Process pool closed")

    def animation_init(self):
        """Animation initialization function"""
        self.scatter._offsets3d = ([], [], [])
        return (self.scatter,)

class RegionController:
    """區域控制器，負責管理特定空間區域內的無人機群組"""
    def __init__(self, region_id, region_bounds, max_drones_per_group=200):
        self.region_id = region_id
        self.bounds = region_bounds  # (min_x, max_x, min_y, max_y, min_z, max_z)
        self.max_drones_per_group = max_drones_per_group
        self.drone_groups = []  # 子群組列表
        self.spatial_grid = SpatialGrid(
            world_size=max(max_x - min_x, max_y - min_y, max_z - min_z)
        )

    def create_drone_groups(self, drones):
        """將區域內的無人機分配到子群組"""
        # 按照空間位置將無人機分組
        drones_in_region = [
            drone for drone in drones
            if self._is_in_region(drone)
        ]

        # 使用K-means聚類將無人機分成子群組
        num_groups = max(1, len(drones_in_region) // self.max_drones_per_group)
        if len(drones_in_region) > 0:
            positions = np.array([[d.x, d.y, d.z] for d in drones_in_region])
            kmeans = KMeans(n_clusters=num_groups).fit(positions)

            # 根據聚類結果創建群組
            groups = [[] for _ in range(num_groups)]
            for drone, label in zip(drones_in_region, kmeans.labels_):
                groups[label].append(drone)

            self.drone_groups = groups

    def _is_in_region(self, drone):
        """檢查無人機是否在該區域內"""
        min_x, max_x, min_y, max_y, min_z, max_z = self.bounds
        return (min_x <= drone.x <= max_x and
                min_y <= drone.y <= max_y and
                min_z <= drone.z <= max_z)

    def update_groups(self):
        """更新所有子群組"""
        for group in self.drone_groups:
            self._update_group(group)

    def _update_group(self, group):
        """更新單個子群組內的無人機"""
        # 更新空間網格
        for drone in group:
            self.spatial_grid.update_drone_position(drone)

        # 更新每個無人機的位置
        for drone in group:
            nearby_drones = self.spatial_grid.get_nearby_drones(drone)
            drone.update_position(nearby_drones)

    def get_group_status(self):
        """獲取區域內所有群組的狀態"""
        status = {
            'region_id': self.region_id,
            'num_groups': len(self.drone_groups),
            'total_drones': sum(len(group) for group in self.drone_groups),
            'groups': [{
                'size': len(group),
                'center': np.mean([[d.x, d.y, d.z] for d in group], axis=0)
            } for group in self.drone_groups]
        }
        return status

class DistributedFormationControl:
    """分布式集群控制系統"""
    def __init__(self, num_drones, world_size=40):
        self.world_size = world_size
        self.num_regions = max(1, num_drones // 1000)  # 每1000台無人機一個區域

        # 創建區域控制器
        self.region_controllers = self._create_region_controllers()

        # 初始化無人機
        self.drones = self._initialize_drones(num_drones)

        # 分配無人機到區域
        self._distribute_drones()

    def _create_region_controllers(self):
        """創建區域控制器"""
        controllers = []
        region_size = self.world_size / np.ceil(np.cbrt(self.num_regions))

        # 在3D空間中均勻分配區域
        for i in range(int(np.ceil(np.cbrt(self.num_regions)))):
            for j in range(int(np.ceil(np.cbrt(self.num_regions)))):
                for k in range(int(np.ceil(np.cbrt(self.num_regions)))):
                    region_bounds = (
                        i * region_size, (i + 1) * region_size,  # x範圍
                        j * region_size, (j + 1) * region_size,  # y範圍
                        k * region_size, (k + 1) * region_size   # z範圍
                    )
                    controller = RegionController(
                        region_id=len(controllers),
                        region_bounds=region_bounds
                    )
                    controllers.append(controller)

                    if len(controllers) >= self.num_regions:
                        return controllers
        return controllers

    def _initialize_drones(self, num_drones):
        """初始化所有無人機"""
        drones = []
        for i in range(num_drones):
            x = np.random.uniform(0, self.world_size)
            y = np.random.uniform(0, self.world_size)
            z = np.random.uniform(0, self.world_size)
            drone = Drone(x, y, z, x, y, z, i)
            drones.append(drone)
        return drones

    def _distribute_drones(self):
        """將無人機分配到各個區域"""
        for controller in self.region_controllers:
            controller.create_drone_groups(self.drones)

    def update_formation(self):
        """更新整個集群的編隊"""
        # 並行更新所有區域
        with ThreadPoolExecutor() as executor:
            executor.map(lambda c: c.update_groups(), self.region_controllers)

    def get_system_status(self):
        """獲取整個系統的狀態"""
        return {
            'total_drones': len(self.drones),
            'num_regions': len(self.region_controllers),
            'regions': [
                controller.get_group_status()
                for controller in self.region_controllers
            ]
        }

    def set_formation(self, formation_type):
        """設置目標編隊"""
        # 計算目標位置
        target_positions = self._calculate_formation_positions(formation_type)

        # 使用匈牙利算法優化分配
        cost_matrix = np.zeros((len(self.drones), len(target_positions)))
        for i, drone in enumerate(self.drones):
            for j, (tx, ty, tz) in enumerate(target_positions):
                cost_matrix[i, j] = np.sqrt(
                    (drone.x - tx)**2 +
                    (drone.y - ty)**2 +
                    (drone.z - tz)**2
                )

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 更新無人機目標位置
        for drone_idx, target_idx in zip(row_ind, col_ind):
            tx, ty, tz = target_positions[target_idx]
            self.drones[drone_idx].target_x = tx
            self.drones[drone_idx].target_y = ty
            self.drones[drone_idx].target_z = tz

        # 重新分配無人機到區域
        self._distribute_drones()

if __name__ == "__main__":
    try:
        formation = FormationControl(num_drones=125)
        formation.animate()
    finally:
        # Ensure process pool is closed when main program exits
        if hasattr(formation, 'pool'):
            formation.pool.terminate()
            formation.pool.join()
