"""
Core drone module for the drone swarm system.
This module contains the base Drone class with movement and collision avoidance capabilities.
"""

import time
import numpy as np

class Drone:
    """Base drone class with movement and collision avoidance capabilities"""

    def __init__(self, x, y, z, target_x, target_y, target_z, id_num, is_aerial_station=False):
        """Initialize a drone with position and movement parameters

        Args:
            x, y, z: Initial position coordinates
            target_x, target_y, target_z: Target position coordinates
            id_num: Unique identifier for the drone
            is_aerial_station: Whether this drone acts as an aerial station
        """
        # Position attributes
        self.x = x
        self.y = y
        self.z = z
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z

        # Identity attributes
        self.id = id_num
        self.is_aerial_station = is_aerial_station
        self.on_ground = True

        # Movement parameters
        self.speed = 0.1  # Basic movement speed
        self.min_speed = 0.01  # Minimum movement speed
        self.acceleration = 1.5  # Acceleration factor
        self.deceleration_distance = 2.0  # Start deceleration distance

        # Collision avoidance parameters
        self.safe_distance = 1.5  # Safe distance between drones
        self.avoid_factor = 0.5  # Collision avoidance strength factor

        # Velocity components
        self.velocity_x = 0  # Current velocity in x direction
        self.velocity_y = 0  # Current velocity in y direction
        self.velocity_z = 0  # Current velocity in z direction
        self.max_speed = 0.5  # Maximum speed limit
        self.max_acceleration = 0.2  # Maximum acceleration limit
        self.last_update_time = time.time()

    def distance_to_target(self):
        """Calculate distance to target position

        Returns:
            float: Euclidean distance to target position
        """
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def limit_velocity(self, vx, vy, vz):
        """Limit velocity magnitude to maximum speed

        Args:
            vx, vy, vz: Velocity components

        Returns:
            tuple: Limited velocity components (vx, vy, vz)
        """
        speed = np.sqrt(vx*vx + vy*vy + vz*vz)
        if speed > self.max_speed:
            factor = self.max_speed / speed
            return vx * factor, vy * factor, vz * factor
        return vx, vy, vz

    def limit_acceleration(self, new_vx, new_vy, new_vz):
        """Limit acceleration magnitude

        Args:
            new_vx, new_vy, new_vz: Target velocity components

        Returns:
            tuple: Velocity components limited by acceleration
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0:
            return new_vx, new_vy, new_vz

        dvx = (new_vx - self.velocity_x) / dt
        dvy = (new_vy - self.velocity_y) / dt
        dvz = (new_vz - self.velocity_z) / dt

        acc = np.sqrt(dvx*dvx + dvy*dvy + dvz*dvz)
        if acc > self.max_acceleration:
            factor = self.max_acceleration / acc
            return (self.velocity_x + dvx * factor * dt,
                    self.velocity_y + dvy * factor * dt,
                    self.velocity_z + dvz * factor * dt)

        self.last_update_time = current_time
        return new_vx, new_vy, new_vz

    def calculate_avoidance_velocity(self, nearby_drones):
        """Calculate collision avoidance velocity components with improved algorithm

        Args:
            nearby_drones: List of nearby drones to avoid

        Returns:
            tuple: Avoidance velocity components (vx, vy, vz)
        """
        avoid_vx, avoid_vy, avoid_vz = 0, 0, 0

        if not nearby_drones:
            return avoid_vx, avoid_vy, avoid_vz

        # Calculate repulsive forces from all nearby drones
        for drone in nearby_drones:
            dx = self.x - drone.x
            dy = self.y - drone.y
            dz = self.z - drone.z
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)

            if distance < self.safe_distance and distance > 0:
                # Enhanced avoidance with exponential decay
                avoid_factor = self.avoid_factor * np.exp(-distance / self.safe_distance)

                # Predict future positions based on current velocities
                future_dx = dx + (self.velocity_x - drone.velocity_x)
                future_dy = dy + (self.velocity_y - drone.velocity_y)
                future_dz = dz + (self.velocity_z - drone.velocity_z)
                future_distance = np.sqrt(future_dx*future_dx + future_dy*future_dy + future_dz*future_dz)

                # Combine current and predicted positions for avoidance
                if future_distance < distance:  # Approaching each other
                    avoid_factor *= 1.5  # Increase avoidance strength

                # Calculate weighted avoidance components
                avoid_vx += (dx/distance * 0.7 + future_dx/future_distance * 0.3) * avoid_factor
                avoid_vy += (dy/distance * 0.7 + future_dy/future_distance * 0.3) * avoid_factor
                avoid_vz += (dz/distance * 0.7 + future_dz/future_distance * 0.3) * avoid_factor

        # Normalize and scale avoidance velocity
        total_magnitude = np.sqrt(avoid_vx*avoid_vx + avoid_vy*avoid_vy + avoid_vz*avoid_vz)
        if total_magnitude > 0:
            scale_factor = min(self.max_speed, total_magnitude) / total_magnitude
            avoid_vx *= scale_factor
            avoid_vy *= scale_factor
            avoid_vz *= scale_factor

        return avoid_vx, avoid_vy, avoid_vz

    def update_position(self, nearby_drones=[]):
        """Update drone position with enhanced movement and collision avoidance

        Args:
            nearby_drones: List of nearby drones to consider for collision avoidance
        """
        # Calculate distance to target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        distance = np.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < 0.01:  # Convergence threshold
            self.x = self.target_x
            self.y = self.target_y
            self.z = self.target_z
            self.velocity_x = self.velocity_y = self.velocity_z = 0
            return

        # Enhanced speed control with smooth acceleration/deceleration
        target_speed = self.speed
        if distance > self.deceleration_distance:
            # Gradual acceleration
            target_speed = min(self.speed * self.acceleration,
                             self.max_speed)
        else:
            # Smooth deceleration using sigmoid function
            decel_factor = 1 / (1 + np.exp(-5 * (distance/self.deceleration_distance - 0.5)))
            target_speed = max(self.min_speed,
                             self.speed * decel_factor)

        # Calculate desired velocity components
        direction_x = dx / distance
        direction_y = dy / distance
        direction_z = dz / distance

        # Calculate target direction velocity with current speed
        new_vx = direction_x * target_speed
        new_vy = direction_y * target_speed
        new_vz = direction_z * target_speed

        # Add collision avoidance velocity
        avoid_vx, avoid_vy, avoid_vz = self.calculate_avoidance_velocity(nearby_drones)
        new_vx += avoid_vx
        new_vy += avoid_vy
        new_vz += avoid_vz

        # Apply velocity and acceleration limits
        new_vx, new_vy, new_vz = self.limit_velocity(new_vx, new_vy, new_vz)
        new_vx, new_vy, new_vz = self.limit_acceleration(new_vx, new_vy, new_vz)

        # Smooth velocity transition
        smoothing = 0.8  # Velocity smoothing factor
        self.velocity_x = smoothing * self.velocity_x + (1 - smoothing) * new_vx
        self.velocity_y = smoothing * self.velocity_y + (1 - smoothing) * new_vy
        self.velocity_z = smoothing * self.velocity_z + (1 - smoothing) * new_vz

        # Update position
        self.x += self.velocity_x
        self.y += self.velocity_y
        self.z += self.velocity_z

        # Update ground status with small buffer
        self.on_ground = (self.z < 0.1)

    def update_from_state(self, state_dict):
        """Update drone state from a dictionary

        Args:
            state_dict: Dictionary containing drone state parameters
        """
        self.x = state_dict['x']
        self.y = state_dict['y']
        self.z = state_dict['z']
        self.on_ground = state_dict['on_ground']
        self.target_x = state_dict['target_x']
        self.target_y = state_dict['target_y']
        self.target_z = state_dict['target_z']
