import numpy as np


def greedy_vrp(dist_matrix):
    n = dist_matrix.shape[0]
    visited = [0]          # start = index 0
    unvisited = list(range(1, n))
    total_cost = 0

    while unvisited:
        current = visited[-1]

        next_target = min(
            unvisited,
            key=lambda x: dist_matrix[current, x]
        )

        total_cost += dist_matrix[current, next_target]
        visited.append(next_target)
        unvisited.remove(next_target)

    return visited, total_cost
