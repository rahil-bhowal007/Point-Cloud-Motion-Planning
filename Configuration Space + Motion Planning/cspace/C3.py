import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from helper_functions import *
import typing


def C3_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, q_grid: np.array, q_goal: np.array) -> np.array:
    """Create a new 2D array that shows the distance from each point in the configuration space to the goal configuration.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        The configuration space of the robot given by C2. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].

    Returns
    -------
    np.array
       A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration. 
       The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    ### Insert your code below: ###

    distances = np.full_like(cspace, np.inf)
    goal_index_1 = np.argmin(np.abs(q_grid - q_goal[0]))
    goal_index_2 = np.argmin(np.abs(q_grid - q_goal[1]))
    distances[goal_index_1, goal_index_2] = 2
    neighbors = np.array(
        [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])
    queue = np.array([[goal_index_1, goal_index_2]])
    while queue.size > 0:
        current = queue[0]
        queue = np.delete(queue, 0, axis=0)

        for i in range(8):
            next_point = current + neighbors[i]
            x, y = next_point

            if 0 <= x < cspace.shape[0] and 0 <= y < cspace.shape[1]:
                if distances[x, y] == np.inf and cspace[x, y] == 0:
                    distances[x, y] = distances[current[0], current[1]] + 1
                    queue = np.vstack((queue, next_point))

    distances[np.isinf(distances)] = 0
    distances[cspace == 1] = 1

    return distances
