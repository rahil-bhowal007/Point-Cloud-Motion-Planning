import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from helper_functions import *
from q2poly import q2poly
from shapely.geometry import Polygon as Polygon_shapely
import typing


def plot_polygon(polygon, ax, color, alpha=1.0):
    if isinstance(polygon, Polygon_shapely):
        x, y = polygon.exterior.xy
        ax.fill(x, y, color=color, alpha=alpha)
    else:
        x, y = zip(*polygon)
        ax.fill(x, y, color=color, alpha=alpha)


def C6_func(robot: typing.Dict[str, typing.List[float]], q_path: typing.List[np.array], obstacles: typing.List[Polygon]) -> int:
    """Calculate the number of collisions that occur along the path.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q_path : typing.List[np.array]
       A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles.

    Returns
    -------
    int
        The number of collisions that occur along the path.
    """

    ### Insert your code below: ###
    num_collisions = 0

    fig, ax = plt.subplots()

    for i in range(1, len(q_path)):
        q1 = q_path[i - 1]
        q2 = q_path[i]
        poly11, poly12, _, _ = q2poly(robot, q1)
        poly21, poly22, _, _ = q2poly(robot, q2)
        coords11 = np.array(poly11.exterior.coords)
        coords12 = np.array(poly12.exterior.coords)
        coords21 = np.array(poly21.exterior.coords)
        coords22 = np.array(poly22.exterior.coords)
        arm1_space = np.array(Polygon_shapely(
            np.vstack((coords11, coords21))).convex_hull.exterior.coords.xy).T
        arm2_space = np.array(Polygon_shapely(
            np.vstack((coords12, coords22))).convex_hull.exterior.coords.xy).T
        in_collision = False
        for obstacle in obstacles:
            plot_polygon(obstacle, ax, 'k')
            if Polygon_shapely(obstacle).intersects(Polygon_shapely(arm1_space)) or Polygon_shapely(obstacle).intersects(Polygon_shapely(arm2_space)):
                in_collision = True
                break
        if in_collision:
            num_collisions += 1
            plot_polygon(arm1_space, ax, 'r', alpha=0.3)
            plot_polygon(arm2_space, ax, 'b', alpha=0.3)
    plt.title("Swept Volume collisions")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()
    return num_collisions
