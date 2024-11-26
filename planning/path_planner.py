"""Path planning module for drone swarm."""
import numpy as np
from sklearn.cluster import KMeans

class PathPlanner:
    def __init__(self, max_speed=0.5):
        self.max_speed = max_speed

    def optimize_assignments(self, current_positions, target_positions, num_clusters=5):
        """Optimize drone assignments to minimize total distance"""
        current_array = np.array(current_positions)
        target_array = np.array(target_positions)

        # Use K-means to cluster current positions
        kmeans = KMeans(n_clusters=min(num_clusters, len(current_positions)))
        clusters = kmeans.fit_predict(current_array)

        # Assign within clusters
        assignment = []
        for cluster_id in range(kmeans.n_clusters):
            cluster_current = current_array[clusters == cluster_id]
            cluster_indices = np.where(clusters == cluster_id)[0]

            # Find closest target positions for this cluster
            distances = np.linalg.norm(cluster_current[:, np.newaxis] - target_array, axis=2)
            min_dist_indices = np.argmin(distances, axis=1)

            for orig_idx, target_idx in zip(cluster_indices, min_dist_indices):
                assignment.append((orig_idx, target_idx))

        return [x[1] for x in sorted(assignment)]

    def calculate_step(self, current_pos, target_pos, dt):
        """Calculate next position step towards target"""
        direction = np.array(target_pos) - np.array(current_pos)
        distance = np.linalg.norm(direction)

        if distance < 0.01:  # If very close to target
            return target_pos

        # Normalize direction and apply speed limit
        speed = min(distance / dt, self.max_speed)
        normalized_direction = direction / distance
        movement = normalized_direction * speed * dt

        # Calculate new position
        return tuple(np.array(current_pos) + movement)
