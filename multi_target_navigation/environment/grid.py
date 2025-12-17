import numpy as np
import matplotlib.pyplot as plt

from multi_target_navigation.algorithms.bfs import BFSPlanner
from multi_target_navigation.algorithms.astar import AStarPlanner



class GridEnvironment:
    def __init__(self, width, height, start, targets, obstacles=None):
        self.width = width
        self.height = height
        self.start = start              # (x, y)
        self.targets = targets          # [(x1, y1), (x2, y2), ...]
        self.obstacles = obstacles if obstacles else []

        self.grid = np.zeros((height, width))
        self._build_grid()

    def _build_grid(self):
        # Obstacles
        for (x, y) in self.obstacles:
            self.grid[y, x] = -1

        # Targets
        for (x, y) in self.targets:
            self.grid[y, x] = 2

        # Start
        sx, sy = self.start
        self.grid[sy, sx] = 1

    def visualize(self, path=None, label="Path", color="white"):
       plt.figure(figsize=(6, 6))
       plt.imshow(self.grid, cmap="tab10")
       plt.grid(True)
       plt.xticks(range(self.width))
       plt.yticks(range(self.height))

       # Start
       plt.scatter(*self.start, c="green", s=200, label="Start (Depot)")

       # Targets
       for i, (x, y) in enumerate(self.targets):
          plt.scatter(x, y, c="red", s=150)
          plt.text(x + 0.1, y + 0.1, f"T{i}", color="black")

       # Obstacles
       if self.obstacles:
          ox, oy = zip(*self.obstacles)
          plt.scatter(ox, oy, c="black", s=100, label="Obstacle")

       # Path
       if path:
          px, py = zip(*path)
          plt.plot(px, py, color=color, linewidth=3, label=label)

       plt.legend()
       plt.title("Multi-Target Grid Environment")
       plt.show()



if __name__ == "__main__":
    env = GridEnvironment(
        width=10,
        height=10,
        start=(1, 1),
        targets=[(7, 2), (8, 8), (2, 7)],
        obstacles=[(4, 4), (4, 5), (5, 4)]
    )

    goal = env.targets[0]

    bfs = BFSPlanner(env.grid, env.start, goal)
    bfs_path = bfs.search()

    astar = AStarPlanner(env.grid, env.start, goal)
    astar_path = astar.search()

    env.visualize(bfs_path, label="BFS Path", color="white")
    env.visualize(astar_path, label="A* Path", color="yellow")

