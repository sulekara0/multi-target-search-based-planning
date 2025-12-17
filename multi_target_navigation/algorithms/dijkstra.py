import heapq


class DijkstraPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.height = grid.shape[0]
        self.width = grid.shape[1]

    def get_neighbors(self, node):
        x, y = node
        neighbors = []

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny, nx] != -1:
                    neighbors.append((nx, ny))
        return neighbors

    def search(self):
        pq = []
        heapq.heappush(pq, (0, self.start))

        came_from = {self.start: None}
        cost_so_far = {self.start: 0}

        while pq:
            current_cost, current = heapq.heappop(pq)

            if current == self.goal:
                return self.reconstruct_path(came_from)

            for neighbor in self.get_neighbors(current):
                new_cost = current_cost + 1

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor))
                    came_from[neighbor] = current

        return None

    def reconstruct_path(self, came_from):
        path = []
        current = self.goal
        while current:
            path.append(current)
            current = came_from[current]
        return path[::-1]
