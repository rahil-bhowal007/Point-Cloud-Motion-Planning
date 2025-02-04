import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as Polygon_shapely
from helper_functions import *
from q2poly import q2poly
import typing


def C2_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, obstacles: typing.List[Polygon], q_grid: np.array) -> np.array:
    """Create the configuration space for the robot with the given obstacles in the given empty cspace array.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        An empty 2D numpy array
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.

    Returns
    -------
    np.array
        A 2D numpy array representing the updated configuration space. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    ### Insert your code below: ###

    cspace = np.zeros((len(q_grid), len(q_grid)))
    obstacles = [Polygon_shapely(obstacle) if not isinstance(
        obstacle, Polygon_shapely) else obstacle for obstacle in obstacles]
    for i1 in range(len(q_grid)):
        for i2 in range(len(q_grid)):
            q1 = q_grid[i1]
            q2 = q_grid[i2]
            poly1, poly2, _, _ = q2poly(robot, [q1, q2])
            in_collision = False
            for obstacle in obstacles:
                if poly1.union(poly2).intersects(obstacle):
                    in_collision = True
                    break
            cspace[i1, i2] = 1 if in_collision else 0

    return cspace
