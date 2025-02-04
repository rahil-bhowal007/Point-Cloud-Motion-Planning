import numpy as np
from robot import Simple_Manipulator as Robot


def point_line_distance(pt, start, end):

    line_vec = end - start

    pt_vec = pt - start

    projection = np.dot(pt_vec, line_vec) / np.dot(line_vec, line_vec)

    projection = max(0, min(projection, 1))

    closest = start + projection * line_vec

    distance = np.linalg.norm(pt - closest)
    return distance


def simplify_path(start_idx, end_idx, path, tolerance=0.1):
    if start_idx == end_idx:
        return [path[start_idx]]

    max_distance = 0
    farthest_idx = 0

    for i in range(start_idx + 1, end_idx):
        distance = point_line_distance(
            path[i], path[start_idx], path[end_idx])
        if distance > max_distance:
            max_distance = distance
            farthest_idx = i

    if max_distance < tolerance:
        return [path[start_idx], path[end_idx]]

    return simplify_path(start_idx, farthest_idx, path) + simplify_path(farthest_idx, end_idx, path)


def M5(robot: Robot, path: np.array) -> np.array:
    """Smooth the given path

    Parameters
    ----------
    robot : Robot
        our robot object
    path : np.array
        Nx4 numpy array containing a collision-free path between q_start and q_goal

    Returns
    -------
    np.array
        Nx4 numpy array containing a smoothed version of the
        input path, where some unnecessary intermediate
        waypoints may have been removed
    """

    # student work start here

    simplified_path = simplify_path(0, len(path) - 1, path)

    path = np.array(simplified_path)
    return path
