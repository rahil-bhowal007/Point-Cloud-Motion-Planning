import typing
import numpy as np
from networkx import Graph
from robot import Simple_Manipulator as Robot

from M1 import M1


def M2(robot: Robot, num_samples: int, num_neighbors: int) -> typing.Tuple[np.array, Graph]:
    """ Implement the PRM algorithm

    Parameters
    ----------
    robot : Robot
        our pybullet robot class
    num_samples : int
        number of samples in PRM
    num_neighbors : int
        number of closest neighbors to consider in PRM

    Returns
    -------
    typing.Tuple[np.array, Graph]
        np.array: 
            num_samples x 4 numpy array, sampled configurations in the roadmap (vertices)
        G: 
            a NetworkX graph object with weighted edges indicating the distance between connected nodes in the joint configuration space.
            This should be impelemented as an undirected graph.
    """

    # HINTS
    # useful functions and parameters
    # robot.lower_lims, robot.upper_lims -> Joint Limits
    # robot.check_edge() -> check the linear path between 2 joint configurations for collisions

    # student code start here

    G = Graph()

    samples = M1(robot.lower_lims, robot.upper_lims, num_samples)

    for i, sample in enumerate(samples):
        G.add_node(i, config=sample)

    for i, sample in enumerate(samples):

        distances = np.linalg.norm(samples - sample, axis=1)

        nearest_indices = np.argsort(distances)[1:num_neighbors+1]
        for neighbor_index in nearest_indices:
            neighbor_config = samples[neighbor_index]

            if robot.check_edge(sample, neighbor_config):
                G.add_edge(i, neighbor_index, weight=distances[neighbor_index])

    return samples, G
