from collections import deque

class BFSPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.height = grid.shape[0]
        self.width = grid.shape[1]
        self.nodes_visited = 0  # ANALİZ İÇİN EKLENDİ

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        # 4 Yönlü hareket
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny, nx] != -1:
                    neighbors.append((nx, ny))
        return neighbors

    def search(self):
        queue = deque([self.start])
        came_from = {self.start: None}
        self.nodes_visited = 0

        while queue:
            current = queue.popleft()
            self.nodes_visited += 1  # SAYAÇ ARTIRMA

            if current == self.goal:
                return self.reconstruct_path(came_from)

            for neighbor in self.get_neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    queue.append(neighbor)

        return None

    def reconstruct_path(self, came_from):
        path = []
        current = self.goal
        while current:
            path.append(current)
            current = came_from[current]
        return path[::-1]
    