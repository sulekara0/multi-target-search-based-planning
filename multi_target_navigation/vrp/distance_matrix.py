import numpy as np

from multi_target_navigation.algorithms.bfs import BFSPlanner
from multi_target_navigation.algorithms.dijkstra import DijkstraPlanner
from multi_target_navigation.algorithms.astar import AStarPlanner


def compute_distance(grid, points, method="astar"):
    n = len(points)
    dist_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            if i == j:
                continue

            start = points[i]
            goal = points[j]

            if method == "bfs":
                planner = BFSPlanner(grid, start, goal)
            elif method == "dijkstra":
                planner = DijkstraPlanner(grid, start, goal)
            else:
                planner = AStarPlanner(grid, start, goal)

            path = planner.search()
            dist_matrix[i, j] = len(path) - 1 if path else np.inf

    return dist_matrix
