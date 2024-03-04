import numpy as np
import matplotlib.pyplot as plt
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0  # Cost to reach this node from the start node

class RRTStar:
    def __init__(self, start, goal, obstacle_grid, grid_resolution, robot_radius, max_iterations=10000, step_size=1, neighbor_radius=15):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacle_grid = obstacle_grid
        self.grid_resolution = grid_resolution
        self.robot_radius = robot_radius
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.neighbor_radius = neighbor_radius
        self.nodes = [self.start]


    
    def distance(self, a, b):
        return np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
    

    def nearest_node(self, n):
        return min(self.nodes, key=lambda node: self.distance(node, n))

    def near_nodes(self, new_node):
        return [node for node in self.nodes if self.distance(node, new_node) <= self.neighbor_radius]

    def is_collision_free(self, node_a, node_b):
        def line_points(x0, y0, x1, y1):
            """Bresenham's Line Algorithm to produce points on a line"""
            points = []
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            x, y = x0, y0
            sx = -1 if x0 > x1 else 1
            sy = -1 if y0 > y1 else 1
            if dx > dy:
                err = dx / 2.0
                while x != x1:
                    points.append((x, y))
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx
            else:
                err = dy / 2.0
                while y != y1:
                    points.append((x, y))
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy
            points.append((x, y))
            return points

        x0, y0 = int(node_a.x / self.grid_resolution), int(node_a.y / self.grid_resolution)
        x1, y1 = int(node_b.x / self.grid_resolution), int(node_b.y / self.grid_resolution)

        for x, y in line_points(x0, y0, x1, y1):
            # Check if the point is within the bounds of the grid
            if x < 0 or y < 0 or x >= self.obstacle_grid.shape[0] or y >= self.obstacle_grid.shape[1]:
                return False
            # Check if the point is an obstacle
            if self.obstacle_grid[x, y] == 1:
                return False
        return True


    def generate_path(self):
        for _ in range(self.max_iterations):
            random_node = Node(random.uniform(0, self.obstacle_grid.shape[0]*self.grid_resolution), random.uniform(0, self.obstacle_grid.shape[1]*self.grid_resolution))
            nearest = self.nearest_node(random_node)

            dx = random_node.x - nearest.x
            dy = random_node.y - nearest.y
            dist = np.sqrt(dx**2 + dy**2)
            dx, dy = dx/dist, dy/dist

            new_node = Node(nearest.x + dx * self.step_size, nearest.y + dy * self.step_size)

            if self.is_collision_free(nearest, new_node):
                nears = self.near_nodes(new_node)
                min_cost = nearest.cost + self.distance(nearest, new_node)
                new_node.parent = nearest
                new_node.cost = min_cost

                for near_node in nears:
                    if self.is_collision_free(near_node, new_node):
                        cost = near_node.cost + self.distance(near_node, new_node)
                        if cost < min_cost:
                            new_node.parent = near_node
                            new_node.cost = cost

                self.nodes.append(new_node)

                for near_node in nears:
                    if self.is_collision_free(new_node, near_node):
                        cost = new_node.cost + self.distance(new_node, near_node)
                        if cost < near_node.cost:
                            near_node.parent = new_node
                            near_node.cost = cost

                if self.distance(new_node, self.goal) < self.step_size:
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    break

        path = []
        current = self.goal
        while current.parent is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.append((self.start.x, self.start.y))

        return path[::-1]

# Example usage
# start = (0, 0)  # Start position as a tuple
# goal = (100, 100)  # Goal position as a tuple
# obstacle_grid = np.zeros((100, 100))  # Example grid
# obstacle_grid[30:70, 40:60] = 1  # Example obstacle
# grid_resolution = 1
# robot_radius = 5

# rrt_star = RRTStar(start, goal, obstacle_grid, grid_resolution, robot_radius)
# path = rrt_star.generate_path()
