import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import typing

def plot_obstacles_robot(obstacles: typing.List[Polygon],link1: typing.List[typing.List[float]],link2: typing.List[typing.List[float]], origin1: typing.List[float],origin2: typing.List[float]) -> None:
    """A function to plot the robot and the obstacles

    Parameters
    ----------
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    link1 : typing.List[typing.List[float]]
        A list of 2-element lists representing the vertices of the first link
    link2 : typing.List[typing.List[float]]
        A list of 2-element lists representing the vertices of the second link
    origin1 : typing.List[float]
        A 2-element list representing the pivot of the first link
    origin2 : typing.List[float]
        A 2-element list representing the pivot of the second link
    """

    plt.figure()
    plt.xlim([0, 12])
    plt.ylim([0, 12])

    plt.axis('square')
    
    for i in range(len(obstacles)):
        coord = obstacles[i]
        p = Polygon(coord, facecolor = 'k')
        plt.gca().add_patch(p)
    
    link1_p = Polygon(link1, facecolor = 'r')
    plt.gca().add_patch(link1_p)

    link2_p = Polygon(link2, facecolor = 'b')
    plt.gca().add_patch(link2_p)

    plt.plot(origin1[0], origin1[1], 'k.', markersize = 10)
    plt.plot(origin2[0], origin2[1], 'k.', markersize = 10)
    