def build_full_path(env, order, planner_class):
    full_path = []
    points = [env.start] + env.targets

    for i in range(len(order) - 1):
        start = points[order[i]]
        goal = points[order[i + 1]]

        planner = planner_class(env.grid, start, goal)
        path = planner.search()

        if path is None or len(path) == 0:
            print(f"Path not found from {start} to {goal}")
            continue

        if i > 0:
            path = path[1:]

        full_path.extend(path)

    return full_path
