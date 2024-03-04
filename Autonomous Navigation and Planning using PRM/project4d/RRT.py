import numpy as np
import random
from scipy.ndimage import binary_dilation

class Node:
    def __init__(self, point):
        self.point = point
        self.parent = None

class RRT:
    def __init__(self, occupancy_grid, start, goal, step_size= 0.3, max_iter=40000,grid_resolution=1.0, radius = 0):
        self.occupancy_grid = occupancy_grid
        self.start = start
        self.goal = goal
        self.step_size = step_size
        self.max_iter = max_iter
        self.grid_resolution = grid_resolution
        self.radius = radius

    @staticmethod
    def distance(point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def steer(self, from_node, to_point, step_size):
        from_point = np.array(from_node.point)
        to_point = np.array(to_point)
        direction = to_point - from_point
        length = np.linalg.norm(direction)
        direction = direction / length

        # Ensure the step size does not exceed the distance to the to_point
        step = min(step_size, length)

        # Calculate new point
        new_point = from_point + step * direction


        # Clamp the new point within the grid boundaries
        grid_max_x = (len(self.occupancy_grid[0]) - 1) * self.grid_resolution
        grid_max_y = (len(self.occupancy_grid) - 1) * self.grid_resolution
        new_point[0] = min(max(new_point[0], 0), grid_max_x)
        new_point[1] = min(max(new_point[1], 0), grid_max_y)

        return new_point.tolist()


    def is_collision_free(self, from_point, to_point):
        from_point = np.array(from_point)
        to_point = np.array(to_point)
        distance = np.linalg.norm(to_point - from_point)
        direction = (to_point - from_point) / distance

        num_steps = int(np.ceil(distance / self.grid_resolution))
        for step in range(num_steps + 1):
            point = from_point + direction * step * self.grid_resolution
            if not self.is_collision(point):
                return False  # Collision detected

        return True




    def is_point_free(self, point):
        x_center, y_center = int(point[0] / self.grid_resolution), int(point[1] / self.grid_resolution)
        radius_in_cells = int(self.radius / self.grid_resolution)

        for x in range(x_center - radius_in_cells, x_center + radius_in_cells + 1):
            for y in range(y_center - radius_in_cells, y_center + radius_in_cells + 1):
                if not (0 <= x < len(self.occupancy_grid[0]) and 0 <= y < len(self.occupancy_grid)):
                    return False  # Out of bounds
                if self.occupancy_grid[y][x] != 0:  # Assuming 0 is free space
                    return False  # Collision

        return True
    
    def dilate_obstacles(self, radius):
        # Convert radius to grid cells
        radius_in_cells = int(np.ceil(radius / self.grid_resolution))
        
        # Create a structuring element for dilation
        struct = np.ones((2 * radius_in_cells + 1, 2 * radius_in_cells + 1))
        
        # Perform binary dilation
        self.dilated_occupancy_grid = binary_dilation(self.occupancy_grid, structure=struct).astype(self.occupancy_grid.dtype)

    def is_collision(self, point):
        x, y = point
        grid_y, grid_x = int(y / self.grid_resolution), int(x / self.grid_resolution)

        # Check collision against dilated occupancy grid
        if self.dilated_occupancy_grid[grid_y][grid_x] == 1:  # Assuming 1 represents an obstacle
            return True
        return False



    def nearest_node(self, node_list, point):
        return min(node_list, key=lambda node: self.distance(node.point, point))

    def extract_path(self, goal_node):
        path = []
        current_node = goal_node
        while current_node.parent is not None:
            path.append(current_node.point)
            current_node = current_node.parent
        path.append(current_node.point)
        return path[::-1]

    def plan_path(self):
        # if not self.is_point_free(self.start) or not self.is_point_free(self.goal):
        #     print("Start or goal point is outside the occupancy grid or in an obstacle.")
        #     return None
        self.dilate_obstacles(self.radius) 
        self.step_size = max(self.grid_resolution, self.step_size)
        start_node = Node(self.start)
        goal_node = Node(self.goal)
        nodes = [start_node]
        # print(f'Inside the plan path {len(self.occupancy_grid[0]) = }')
        # print(f'Inside the plan path {len(self.occupancy_grid) = }')

        for _ in range(self.max_iter):
            sample_x = random.uniform(0, (len(self.occupancy_grid[0]) - 1) * self.grid_resolution)
            sample_y = random.uniform(0, (len(self.occupancy_grid) - 1) * self.grid_resolution)
            sample_point = [sample_x, sample_y]
            nearest = self.nearest_node(nodes, sample_point)
            new_point = self.steer(nearest, sample_point, self.step_size)

            if self.is_collision_free(nearest.point, new_point):
                new_node = Node(new_point)
                new_node.parent = nearest
                nodes.append(new_node)

                if self.distance(new_point, self.goal) <= self.step_size:
                    goal_node.parent = new_node
                    return self.extract_path(goal_node)

        return None  # Goal not reached

