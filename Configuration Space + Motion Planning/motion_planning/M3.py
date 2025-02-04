from networkx import Graph, shortest_path
import numpy as np
from robot import Simple_Manipulator as Robot
import typing
from M1 import M1


def sample_configurations(robot, num_samples):
    samples = []
    for _ in range(num_samples):
        sample = M1(robot.lower_lims, robot.upper_lims, num_samples)
        samples.append(sample)
    return np.array(samples)


def generate_adjacency(samples):
    num_samples = len(samples)
    adjacency = np.zeros((num_samples, num_samples))
    for i in range(num_samples):
        for j in range(i + 1, num_samples):
            adjacency[i, j] = np.linalg.norm(samples[i] - samples[j])
            adjacency[j, i] = adjacency[i, j]
    return adjacency


def M3(robot: Robot, samples: np.array, G: Graph, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """ Find a path from q_start to q_goal using the PRM roadmap

    Parameters
    ----------
    robot : Robot
        our robot object
    samples : np.array
        num_samples x 4 numpy array of nodes/vertices in the roadmap
    G : Graph
        An undirected NetworkX graph object with the number of nodes equal to num_samples, 
        and weighted edges indicating collision free connections in the robot's configuration space
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

    # student code start here
    num_samples = 100

    adjacency = generate_adjacency(samples)

    G = Graph()
    for i in range(num_samples):
        for j in range(i + 1, num_samples):
            if adjacency[i, j] != 0:
                G.add_edge(i, j, weight=adjacency[i, j])

    try:

        node_path = shortest_path(G, source=0, target=num_samples - 1)
        path = np.vstack((q_start, samples[node_path], q_goal))
        path_found = True
    except:

        path = np.array([])
        path_found = False

    return path, path_found
