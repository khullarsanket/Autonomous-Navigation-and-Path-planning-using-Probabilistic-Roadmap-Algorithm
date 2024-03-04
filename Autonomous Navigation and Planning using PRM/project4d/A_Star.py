from queue import PriorityQueue
import csv

# result = []
class AStar:
    
    def __init__(self,grid_matrix,grid_resolution, start, goal):
        self.grid_matrix = grid_matrix
        self.grid_resolution = grid_resolution
        self.start_x = int((start[0]/self.grid_resolution))
        self.start_y = int((start[1]/self.grid_resolution))
        self.start = (self.start_x, self.start_y)
        # self.start = (start[0], start[1])

        self.goal_x = int((goal[0]/self.grid_resolution))
        self.goal_y = int((goal[1]/self.grid_resolution))
        self.goal = (self.goal_x, self.goal_y)
        # self.goal = (goal[0], goal[1])

        self.goal_found = False
        self.parent_node = {}
        self.distance = {}
        self.visited = set()
        self.queue = PriorityQueue()
        self.actions = [(-1, 0),
                        (0, -1),
                        (1, 0),
                        (0, 1)]
        self.goal_distance = None
        self.result = None
        
    def ManhattanDistance(self, current_node):
        ## creating the Heuristic function, cost(f(x)) = g(x) + h(x)
        manhattan_distance = abs(current_node[0] - self.goal[0]) + abs(current_node[1] - self.goal[1])
        return manhattan_distance

    def find_path(self):


        for i in range(len(self.grid_matrix[0])):
            for j in range(len(self.grid_matrix)):
                self.distance[(i, j)] = float('inf')
        self.distance[self.start] = 0 + self.ManhattanDistance(self.start)
        self.queue.put((self.distance[self.start], self.start))
    #     print(f'The queue is {queue.get()}')
        
        if self.grid_matrix[self.start[1],self.start[0]] == 1:
            self.result = "Start node is an Obstacle!!"
            return print("Start node is an Obstacle!!")

        if self.grid_matrix[self.goal[1],self.goal[0]] == 1:
            self.result = "Goal is an Obstacle!!"
            
            return print("Goal is an Obstacle!!")


        while not self.queue.empty():
            current_distance, current_node = self.queue.get()


            self.visited.add((current_node))



            # Using the actions to search teh neighbouring nodes

            for action in self.actions:
                possible_col = current_node[0] + action[0]
                possible_row = current_node[1] + action[1]

                if (possible_col, possible_row) not in self.visited:
                    if 0 <= possible_row < len(self.grid_matrix) and 0 <= possible_col < len(self.grid_matrix[0]) and self.grid_matrix[possible_row][possible_col] != 1:
                        new_distance = current_distance + 1 + self.ManhattanDistance((possible_col, possible_row))
                        if new_distance < self.distance[(possible_col, possible_row)]:
                            self.distance[(possible_col, possible_row)] = new_distance
                            self.queue.put((new_distance, (possible_col, possible_row)))
                            
                            
                            possible_node = (new_distance, (possible_col, possible_row))
                            self.parent_node[possible_node] = (current_distance, current_node)
    
            if current_node == self.goal:
                self.goal_found = True 
                self.goal_distance = current_distance
                self.result = "Path Found!!!"
                
                print("Path Found!!!")
                print("\nGenerating Path.........\n" )
                path = self.get_shortest_path()
                print(path)
                break            
                            

        if self.goal_found != True:
            self.result = "Path not Found!!"
            print("Path not Found!!")

    def get_shortest_path(self):
        
        route = PriorityQueue()
        
        child_node = (self.goal_distance, self.goal)


        while child_node in self.parent_node:
            route.put((self.parent_node[child_node]))
            child_node = self.parent_node[child_node]
            
            
            
        path = []

        for i in range(0, route.qsize()):

            distance, position = route.get()

            path.append((round((position[0])*self.grid_resolution), round((position[1])*self.grid_resolution)))



        position = self.goal
        path.append((round((self.goal[0])*self.grid_resolution), round((self.goal[1])*self.grid_resolution)))


        return path



