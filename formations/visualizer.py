"""
Formation visualizer module for visualizing drone formations.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List, Tuple

class FormationVisualizer:
    """Visualizer class for drone formations"""

    def __init__(self, world_size: float):
        self.world_size = world_size
        self.view_angle = 0
        self.view_elevation = 30

        # Initialize matplotlib figure
        self.fig = plt.figure(figsize=(16, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')

    def setup_plot(self):
        """Setup the plot with initial settings"""
        self.ax.set_xlim(0, self.world_size)
        self.ax.set_ylim(0, self.world_size)
        self.ax.set_zlim(0, self.world_size)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def update_view(self, dt: float):
        """Update the view angle for rotation effect"""
        self.view_angle += 20 * dt
        self.ax.view_init(self.view_elevation, self.view_angle)

    def plot_drones(self,
                   drone_positions: List[Tuple[float, float, float]],
                   target_positions: List[Tuple[float, float, float]],
                   ground_station_pos: Tuple[float, float, float],
                   aerial_station_positions: List[Tuple[float, float, float]]):
        """Plot all elements in the formation"""
        self.ax.cla()
        self.setup_plot()

        # Plot drones
        xs, ys, zs = zip(*drone_positions)
        self.ax.scatter(xs, ys, zs, c='b', marker='o', label='Drones')

        # Plot target positions
        if target_positions:
            txs, tys, tzs = zip(*target_positions)
            self.ax.scatter(txs, tys, tzs, c='r', marker='x', label='Targets')

        # Plot ground station
        self.ax.scatter(*ground_station_pos, c='g', marker='^', s=200, label='Ground Station')

        # Plot aerial stations
        if aerial_station_positions:
            axs, ays, azs = zip(*aerial_station_positions)
            self.ax.scatter(axs, ays, azs, c='y', marker='s', s=100, label='Aerial Stations')

        self.ax.legend()

    def show(self):
        """Display the plot"""
        plt.show()

    def close(self):
        """Close the plot"""
        plt.close(self.fig)
