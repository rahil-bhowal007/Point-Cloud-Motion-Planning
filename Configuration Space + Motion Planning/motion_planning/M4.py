from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing


def M4(robot: Robot, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """Implement RRT algorithm to find a path from q_start to q_goal

    Parameters
    ----------
    robot : Robot
        our robot object
    q_start : np.array
        1x4 numpy array denoting the start configuration
    q_goal : np.array
        1x4 numpy array denoting the goal configuration

    Returns
    -------
    typing.Tuple[np.array, bool]
        np.array:
            Nx4 numpy array containing a collision-free path between
            q_start and q_goal, if a path is found. The first row
            should be q_start, the final row should be q_goal.
        bool:
            Boolean denoting whether a path was found
    """

    # student work start here
    vertices = [q_start]
    edges = {}
    max_iterations = 100000
    step_size = 0.05
    goal_tolerance = 0.2
    path_found = False
    for it in range(max_iterations):
        print(f"Iteration number: {it}")
        q_rand = M1(robot.lower_lims, robot.upper_lims, 1000)[0]

        nearest_index = np.argmin(np.linalg.norm(
            np.array(vertices) - q_rand, axis=1))
        q_near = vertices[nearest_index]

        direction = (q_rand - q_near) / np.linalg.norm(q_rand - q_near)
        q_new = q_near + step_size * direction

        if robot.check_edge(q_near, q_new):

            vertices.append(q_new)
            edges[len(vertices) - 1] = nearest_index

            if np.linalg.norm(q_new - q_goal) < goal_tolerance:

                vertices.append(q_goal)
                edges[len(vertices) - 1] = len(vertices) - 2
                path_found = True
                break

    if path_found:
        path = [q_goal]
        current_index = len(vertices) - 1
        while current_index != 0:
            current_index = edges[current_index]
            path.append(vertices[current_index])
        path.reverse()
    else:
        path = []

    return np.array(path), path_found
