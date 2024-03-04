import numpy as np
import networkx as nx
from scipy.spatial import KDTree
from queue import PriorityQueue

class PRMPlanner:
    def __init__(self, occupancy_grid, resolution, n_samples=100, k_nearest=10):
        self.occupancy_grid = occupancy_grid
        self.resolution = resolution
        self.n_samples = n_samples
        self.k_nearest = k_nearest

    def is_collision_free(self, p1, p2):
        # Implement collision checking
        x1, y1 = p1
        x2, y2 = p2
        x1, y1 = int(x1 / self.resolution), int(y1 / self.resolution)
        x2, y2 = int(x2 / self.resolution), int(y2 / self.resolution)
        
        # Bresenham's Line Algorithm for grid traversal
        dx = abs(x2 - x1)
        dy = -abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx + dy
        while True:
            if x1 == x2 and y1 == y2:
                break
            if self.occupancy_grid[x1][y1] == 1:  # Occupied cell
                return False
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x1 += sx
            if e2 <= dx:
                err += dx
                y1 += sy
        return True

    def plan_path(self, start, goal):
        # Sampling points
        free_cells = np.argwhere(self.occupancy_grid == 0)
        samples = free_cells[np.random.choice(free_cells.shape[0], self.n_samples, replace=False)]

        # Adding start and goal to the samples
        samples = np.vstack([samples, np.array(start) / self.resolution, np.array(goal) / self.resolution])

        # Build KDTree for efficient nearest neighbor search
        tree = KDTree(samples)

        # Graph construction
        G = nx.Graph()
        for i in range(samples.shape[0]):
            G.add_node(i, pos=samples[i])

        for i in range(samples.shape[0]):
            # Omitting the 'return_distance' argument
            distances, indices = tree.query(samples[i], k=self.k_nearest)
            for j in indices:
                if i != j and self.is_collision_free(samples[i], samples[j]):
                    G.add_edge(i, j)

        # A* path finding
        start_idx = samples.shape[0] - 2
        goal_idx = samples.shape[0] - 1

        try:
            path = nx.astar_path(G, start_idx, goal_idx)
            return [samples[i] * self.resolution for i in path]
        except nx.NetworkXNoPath:
            return None


# # Example usage
# occupancy_grid = np.zeros((100, 100))  # 100x100 grid, all cells are free initially
# occupancy_grid[40:60, 40:60] = 1  # Adding an obstacle

# start = (10, 10)
# goal = (90, 90)
# resolution = 1

# prm_planner = PRMPlanner(occupancy_grid, resolution)
# path = prm_planner.plan_path(start, goal)
# print("Path:", path)
