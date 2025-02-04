import numpy as np
import math
from helper_functions import *
import typing


def C4_func(distances: np.array, q_grid: np.array, q_start: np.array) -> typing.List[np.array]:
    """Using the distance array from C3, find the optimal path from the start configuration to the goal configuration (zero value).

    Parameters
    ----------
    distances : np.array
        A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration.
        This is given by C3 
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.
        Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """

    ### Insert your code below: ###
    n, m = distances.shape
    start_index_1 = np.argmin(np.abs(q_grid - q_start[0]))
    start_index_2 = np.argmin(np.abs(q_grid - q_start[1]))

    path = np.array([[start_index_1, start_index_2]])

    current = np.array([start_index_1, start_index_2])
    while distances[tuple(current)] != 2:

        neighbors = np.array(
            [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])

        min_distance = math.inf
        next_cell = current.copy()

        for i in range(8):
            neighbor = current + neighbors[i]
            x, y = neighbor[0], neighbor[1]
            if 0 <= x < n and 0 <= y < m:
                if distances[x, y] == 2:
                    next_cell = neighbor
                    break
                elif 1 < distances[x, y] < min_distance:
                    min_distance = distances[x, y]
                    next_cell = neighbor

        if np.array_equal(current, next_cell):
            raise ValueError('Unable to find a path to the goal')

        current = next_cell

        path = np.vstack((path, current))

    return path
