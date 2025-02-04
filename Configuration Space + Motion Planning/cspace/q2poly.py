import numpy as np
from shapely.geometry import Polygon as Polygon_shapely
import typing


def q2poly(robot: typing.Dict[str, typing.List[float]], q: typing.List[float]) -> typing.Tuple[np.array, np.array, np.array, np.array]:
    """ A function that takes in the robot's parameters and a configuration and 
    returns the vertices of the robot's links after transformation and the pivot points of the links after transformation

    Parameters
    ----------
    robot : typing.dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    q : typing.List[float]
        A 2-element list representing the configuration of the robot

    Returns
    -------
    typing.Tuple[np.array, np.array, np.array, np.array]
        np.array: 
            a numpy array representing the vertices of the first link of the robot after transformation
        np.array: 
            a numpy array representing the vertices of the second link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the first link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the second link of the robot after transformation
    """

    # Insert your code below: ###i
    link1 = np.array(robot["link1"])
    link2 = np.array(robot["link2"])
    pivot1 = np.array(robot["pivot1"])
    pivot2 = np.array(robot["pivot2"])

    pivot2 = pivot1 + [pivot2[0] * np.cos(q[0]), pivot2[0] * np.sin(q[0])]

    R1 = np.array([[np.cos(q[0]), -np.sin(q[0])],
                   [np.sin(q[0]), np.cos(q[0])]])
    link1_rotated = np.dot(R1, link1.T).T + pivot1

    R2 = np.array([[np.cos(q[1]), -np.sin(q[1])],
                   [np.sin(q[1]), np.cos(q[1])]])
    link2_rotated = np.dot(R2, np.dot(R1, link2.T)).T + pivot2

    poly1 = Polygon_shapely(link1_rotated)
    poly2 = Polygon_shapely(link2_rotated)

    return poly1, poly2, pivot1, pivot2
