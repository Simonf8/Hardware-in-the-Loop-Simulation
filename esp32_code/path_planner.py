"""
Simplified Path Planner for ESP32/MicroPython using Dijkstra's Algorithm
No NumPy dependencies - uses only built-in Python data structures
"""

class PathPlanner:
    def __init__(self):
        # Simple grid representation - 0 = path, 1 = obstacle
        # Adjust this grid to match your actual line course layout
        self.grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 0
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 1
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 2
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 3
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 4
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 5
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 6
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 7
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 8
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 9
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 10
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 11
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 12
        ]
        
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])
        
        # Cost for moving to each cell (can be modified for different terrain)
        self.movement_cost = 1
        self.turn_cost = 1.2  # Slightly higher cost for turns
        
    def is_valid_position(self, row, col):
        """Check if position is within grid bounds and not an obstacle"""
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.grid[row][col] == 0)
    
    def get_neighbors(self, pos):
        """Get valid neighboring positions (up, down, left, right)"""
        row, col = pos
        neighbors = []
        
        # Check 4 directions: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if self.is_valid_position(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def calculate_cost(self, current_pos, next_pos, came_from):
        """Calculate movement cost (can add turn penalties)"""
        base_cost = self.movement_cost
        
        # Add turn penalty if direction changes
        if came_from and came_from in came_from:
            prev_pos = came_from[current_pos]
            if prev_pos:
                # Calculate if this is a turn
                prev_dir = (current_pos[0] - prev_pos[0], current_pos[1] - prev_pos[1])
                curr_dir = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1])
                if prev_dir != curr_dir:
                    base_cost = self.turn_cost
        
        return base_cost
    
    def find_path(self, start, goal):
        """
        Find shortest path using Dijkstra's algorithm
        Returns list of (row, col) tuples from start to goal
        """
        if not self.is_valid_position(start[0], start[1]):
            print(f"Invalid start position: {start}")
            return []
        
        if not self.is_valid_position(goal[0], goal[1]):
            print(f"Invalid goal position: {goal}")
            return []
        
        if start == goal:
            return [start]
        
        # Initialize data structures
        # Using list as priority queue (not as efficient as heapq but works in MicroPython)
        open_set = [(0, start)]  # (cost, position)
        came_from = {}
        cost_so_far = {start: 0}
        
        while open_set:
            # Find node with lowest cost (manual priority queue)
            current_cost, current = min(open_set)
            open_set.remove((current_cost, current))
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Check all neighbors
            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.calculate_cost(current, neighbor, came_from)
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    open_set.append((new_cost, neighbor))
        
        print("No path found!")
        return []
    
    def print_grid_with_path(self, path):
        """Debug function to visualize the grid and path"""
        if not path:
            print("No path to display")
            return
        
        # Create a copy of the grid for display
        display_grid = []
        for row in self.grid:
            display_grid.append(row[:])  # Copy row
        
        # Mark path on display grid
        for i, (row, col) in enumerate(path):
            if i == 0:
                display_grid[row][col] = 'S'  # Start
            elif i == len(path) - 1:
                display_grid[row][col] = 'G'  # Goal
            else:
                display_grid[row][col] = '*'  # Path
        
        # Print grid
        print("Grid with path (* = path, S = start, G = goal, 1 = obstacle):")
        for row in display_grid:
            print(''.join(str(cell) for cell in row))
    
    def get_path_directions(self, path):
        """Convert path to movement directions"""
        if len(path) < 2:
            return []
        
        directions = []
        for i in range(len(path) - 1):
            current = path[i]
            next_pos = path[i + 1]
            
            dr = next_pos[0] - current[0]
            dc = next_pos[1] - current[1]
            
            if dr == -1:  # Moving up
                directions.append('up')
            elif dr == 1:  # Moving down
                directions.append('down')
            elif dc == -1:  # Moving left
                directions.append('left')
            elif dc == 1:  # Moving right
                directions.append('right')
        
        return directions

# Test function for debugging
def test_pathfinder():
    """Test the pathfinder with a simple example"""
    planner = PathPlanner()
    
    start = (0, 0)
    goal = (12, 16)
    
    print(f"Finding path from {start} to {goal}")
    path = planner.find_path(start, goal)
    
    if path:
        print(f"Path found with {len(path)} steps:")
        print(path)
        directions = planner.get_path_directions(path)
        print(f"Directions: {directions}")
        planner.print_grid_with_path(path)
    else:
        print("No path found!")

# Uncomment to test
# test_pathfinder()