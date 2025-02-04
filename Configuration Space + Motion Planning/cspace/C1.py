import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from q2poly import q2poly
from helper_functions import *
import typing

def C1_func(robot: typing.Dict[str, typing.List[float]], q: typing.List[float], obstacles: typing.List[Polygon]) -> None:
    """ Plot the robot in the workspace.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q : typing.List[float]
        A 2-element list representing the configuration of the robot
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    """

    ### Insert your code below:

    # example of [0,0] configuration
    shape1 = [[5.2, 3  ],\
              [5.2, 2  ],\
              [8.7, 2.1],\
              [8.7, 2.9]]
    shape2 = [[ 8.2,  2.9],\
              [ 8.2,  2.1],\
              [11.2,  2.3],\
              [11.2,  2.7]]
    pivot1 =  [6.4, 2.5]
    pivot2 =  [8.5, 2.5]
    shape1, shape2, pivot1, pivot2 = q2poly(robot, q)
    plot_obstacles_robot(obstacles=obstacles, link1=shape1.exterior.coords, link2=shape2.exterior.coords, origin1=pivot1, origin2=pivot2)

 