import numpy as np
import random


def create_grid(width=20, height=20):
    """
    2D grid oluşturur.
    0 -> boş hücre
    """
    return np.zeros((height, width), dtype=int)


def add_obstacles(grid, obstacle_ratio=0.2):
    """
    Grid üzerine rastgele engeller ekler.
    1 -> engel
    """
    height, width = grid.shape
    obstacle_count = int(height * width * obstacle_ratio)

    placed = 0
    while placed < obstacle_count:
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)

        if grid[y, x] == 0:
            grid[y, x] = 1
            placed += 1

    return grid


def is_valid_cell(grid, x, y):
    """
    Hücre geçerli mi kontrol eder:
    - Grid sınırları içinde mi?
    - Engel değil mi?
    """
    height, width = grid.shape

    if x < 0 or x >= width or y < 0 or y >= height:
        return False

    if grid[y, x] == 1:
        return False

    return True