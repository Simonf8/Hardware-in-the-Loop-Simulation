"""
Path planner for line following robot using Dijkstra's Algorithm.
This is used to find optimal paths through a grid representation of the line following course.
"""
import heapq

class PathPlanner:
    def __init__(self):
        # Grid size for this specific line following course
        self.GRID_ROWS = 13
        self.GRID_COLS = 17
        self.dynamic_obstacles = set()  # Set to store coordinates of dynamic obstacles
        self.robot_position = (0, 0)    # Current robot position in grid coordinates
        self.reset_grid()

    def update_position(self, x, y, theta):
        """Update robot's position based on encoder data"""
        # Convert real-world coordinates to grid coordinates
        grid_x = int(x * 10)  # Assuming 1 grid cell = 0.1 meters
        grid_y = int(y * 10)
        
        if 0 <= grid_x < self.GRID_ROWS and 0 <= grid_y < self.GRID_COLS:
            self.robot_position = (grid_x, grid_y)
            return True
        return False

    def update_obstacles(self, obstacle_data):
        """Update dynamic obstacles based on sensor data
        Args:
            obstacle_data: String of '0's and '1's for left, front, right sensors
        """
        self.dynamic_obstacles.clear()  # Clear old dynamic obstacles
        
        # Calculate potential obstacle positions based on sensor data and robot position
        x, y = self.robot_position
        if len(obstacle_data) >= 3:
            if obstacle_data[0] == '1':  # Left obstacle
                self.dynamic_obstacles.add((x, y-1))
            if obstacle_data[1] == '1':  # Front obstacle
                self.dynamic_obstacles.add((x-1, y))
            if obstacle_data[2] == '1':  # Right obstacle
                self.dynamic_obstacles.add((x, y+1))

    def reset_grid(self):
        """Initialize the grid representation of the course"""
        self.grid = [
            [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]
        ]
        # Cost map - we'll give turns slightly higher cost to prefer straight paths
        self.costs = [[1] * self.GRID_COLS for _ in range(self.GRID_ROWS)]
        # Add higher costs at turns and intersections
        for i in range(1, self.GRID_ROWS-1):
            for j in range(1, self.GRID_COLS-1):
                if self.is_intersection(i, j):
                    self.costs[i][j] = 2

    def is_intersection(self, row, col):
        """Check if a point is an intersection or turn"""
        if self.grid[row][col] != 0:  # Must be on the line
            return False
            
        neighbors = [
            (row-1, col), (row+1, col),  # Up, Down
            (row, col-1), (row, col+1)   # Left, Right
        ]
        
        valid_neighbors = 0
        for r, c in neighbors:
            if (0 <= r < self.GRID_ROWS and 
                0 <= c < self.GRID_COLS and 
                self.grid[r][c] == 0):
                valid_neighbors += 1
        
        return valid_neighbors > 2  # More than 2 valid paths = intersection

    def find_path(self, start_pos, goal_pos):
        """
        Find shortest path from start to goal using Dijkstra's algorithm
        
        Args:
            start_pos: Tuple of (row, col) for start position
            goal_pos: Tuple of (row, col) for goal position
        
        Returns:
            List of (row, col) tuples forming the path
        """
        if not (0 <= start_pos[0] < self.GRID_ROWS and 
               0 <= start_pos[1] < self.GRID_COLS and
               0 <= goal_pos[0] < self.GRID_ROWS and 
               0 <= goal_pos[1] < self.GRID_COLS):
            print("Invalid start or goal position")
            return []
            
        if (self.grid[start_pos[0]][start_pos[1]] != 0 or 
            self.grid[goal_pos[0]][goal_pos[1]] != 0):
            print("Start or goal position is not on a valid path")
            return []

        # Priority queue for Dijkstra's algorithm
        queue = [(0, start_pos)]
        heapq.heapify(queue)
        
        # Track visited nodes and their parents
        visited = set()
        parents = {start_pos: None}
        distances = {start_pos: 0}
        
        found_goal = False
        
        while queue and not found_goal:
            current_dist, current_pos = heapq.heappop(queue)
            
            if current_pos in visited:
                continue
                
            visited.add(current_pos)
            
            if current_pos == goal_pos:
                found_goal = True
                break
                
            # Check neighbors (up, down, left, right)
            neighbors = [
                (current_pos[0]-1, current_pos[1]),  # Up
                (current_pos[0]+1, current_pos[1]),  # Down
                (current_pos[0], current_pos[1]-1),  # Left
                (current_pos[0], current_pos[1]+1)   # Right
            ]
            
            for next_pos in neighbors:
                if (0 <= next_pos[0] < self.GRID_ROWS and 
                    0 <= next_pos[1] < self.GRID_COLS and 
                    self.grid[next_pos[0]][next_pos[1]] == 0):
                    
                    new_dist = current_dist + self.costs[next_pos[0]][next_pos[1]]
                    
                    if next_pos not in distances or new_dist < distances[next_pos]:
                        distances[next_pos] = new_dist
                        parents[next_pos] = current_pos
                        heapq.heappush(queue, (new_dist, next_pos))
        
        if not found_goal:
            print("No path found from start to goal")
            return []
            
        # Reconstruct path
        path = []
        current = goal_pos
        while current is not None:
            path.append(current)
            current = parents.get(current)
        
        path.reverse()
        return path

    def get_next_move(self, current_pos, goal_pos):
        """
        Get the next movement direction based on current position and goal
        
        Args:
            current_pos: Tuple of (row, col) for current position
            goal_pos: Tuple of (row, col) for goal position
            
        Returns:
            String indicating next movement: 'forward', 'turn_left', 'turn_right', etc.
        """
        path = self.find_path(current_pos, goal_pos)
        
        if len(path) < 2:
            return 'stop'  # No path or already at goal
            
        # Get current and next position
        curr = path[0]
        next_pos = path[1]
        
        # Determine direction based on difference
        row_diff = next_pos[0] - curr[0]
        col_diff = next_pos[1] - curr[1]
        
        if row_diff == -1:  # Moving up
            return 'forward'
        elif row_diff == 1:  # Moving down
            return 'forward'
        elif col_diff == -1:  # Moving left
            return 'turn_left_gentle'
        elif col_diff == 1:  # Moving right
            return 'turn_right_gentle'
        
        return 'stop'
