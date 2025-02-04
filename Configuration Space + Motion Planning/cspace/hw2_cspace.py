'''
STUDENTS SHOULD NOT EDIT THIS FILE

All student work should be in C0.py to C7.py.

USAGE: To test motion planning problems 1 - 7 use:
$ python hw2_cspace.py -q <problem_numbe>
For example, to run problem 0, which commands the robot to the position
defined in C1.py, you should run:
$ python hw2_motion.py -q 1

'''

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as Polygon_shapely
import matplotlib.animation as anim
import argparse


from C1 import C1_func
from C2 import C2_func
from C3 import C3_func
from C4 import C4_func
from C5 import C5_func
from C6 import C6_func
from C7 import C7_func


def plot_space(cspace, path = None):
    plt.figure()
    # rows then column
    plt.imshow(1 - cspace.T)
    plt.xlabel("q1")
    plt.ylabel("q2")
    plt.ylim([0,cspace.shape[1]])
    if path is not None:
        plt.plot(path[:,0],path[:,1],'r')
    plt.show()


def plot_path_visualization(robot, q_path, obstacles):

    fps = 10
    for i in range(len(q_path)):
        C1_func(robot, q_path[i], obstacles)
        plt.draw()
        plt.pause(1.0 / fps)
        plt.close()
    


def hw3_cspace(questionNum,cspace=None):
    if questionNum < 1:
        raise ValueError('Error: please enter a question number as a parameter')

    # Set up obstacles as 2-D polygons
    p1 = [[1.5,1.4], [1.5,0.7], [3.5,0.7], [3.5,1.4],[1.5,1.4]]

    p2 = [[9.1,1.9], [8.3,1.4], [8.3,0.5], [9.8,0.5], [9.8,1.4],[9.1,1.9]]
    
    p3 = [[0.9,7.2], [0.9,3.7], [2.0,3.7], [2.0,4.8], [2.8,4.8], [3.3,5.5], [2.8,6.2], [2.0,6.2], [2.0,7.2],[0.9,7.2]]

    p4 = [[9.0,7.3], [9.0,6.2], [8.3,6.2], [8.3,4.8], [9.0,4.8], [9.0,3.8], [10.0,3.8], [10.0,7.3],[9.0,7.3]]

    p5 = [[6.9,7.1], [6.3,5.6], [6.9,5.6],[6.9,7.1]]
    obstacles = [p1, p2, p3, p4, p5]


    # The 2-DOF 2-link rotational planar robot is defined as follows:
    robot = {}
    # Robot links are 2-D polygons as well
    # Pivot point of link 1, with respect to base frame (at origin)
    robot["pivot1"] = [6.4, 2.5]
    
    # Pivot point of link 2, with respect to frame 1 (at pivot1)
    robot["pivot2"] = [2.1, 0]
    
    # Corners of link 1 polygon, with respect to frame 1
    robot["link1"] = [[-1.2,0.5], [-1.2,-0.5], [2.3,-0.4], [2.3,0.4],[-1.2,0.5]]

    # Corners of link 2 polygon, with respect to frame 2
    robot["link2"] = [[-0.3,0.4], [-0.3,-0.4], [2.7,-0.2], [2.7,0.2],[-0.3,0.4]]

    
    q_start = np.array([0.85, 0.9])  # Strating Configuration
    q_goal = np.array([3.05, 0.05])  # Goal Configuration

    if cspace == None:
        cspace_resolution = 300 # Resolution of configuration space
        cspace = np.zeros((cspace_resolution,cspace_resolution)) # Initialize configuration space
    else:
        cspace_resolution = cspace.shape[0] # Resolution of configuration space
    q_grid = np.linspace(0,2*np.pi,cspace.shape[0]) # Discretized grid of configurations

    if questionNum == 1:
        # Visualize the robot at the start and goal configurations
        C1_func(robot, q_start, obstacles)
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.xlabel("x")
        plt.ylabel("y") 
        plt.show()

        C1_func(robot, q_goal, obstacles)
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.xlabel("x")
        plt.ylabel("y") 
        plt.show()
        plt.show()

    elif questionNum == 2:
        # Construct and visualize the configuration space
        cspace = C2_func(robot,cspace,obstacles,q_grid)
        np.save('cspace_'+str(cspace_resolution)+'.npy',cspace)
        plot_space(cspace)
    

    elif questionNum == 3:
        # Construct and visualize the distance transform
    
        if np.sum(cspace) == 0:
            cspace = C2_func(robot,cspace,obstacles,q_grid)
        
        distances = C3_func(robot,cspace,q_grid,q_goal)

        plot_space(distances)
     
    
    elif questionNum == 4:
        # Find the index path from start to goal configuration

        if np.sum(cspace) == 0:
            cspace = C2_func(robot,cspace,obstacles,q_grid)
        
        distances = C3_func(robot,cspace,q_grid,q_goal)
        path = C4_func(distances, q_grid, q_start)
        path = np.array(path)

        plot_space(distances, path)
    

    if questionNum == 5:
        # Convert the index path to a configuration path

        if np.sum(cspace) == 0:
            cspace = C2_func(robot,cspace,obstacles,q_grid)
        
        distances = C3_func(robot,cspace,q_grid,q_goal)
        path = C4_func(distances, q_grid, q_start)
        
        q_path = C5_func(q_grid, q_start, q_goal, path)

        plot_path_visualization(robot, q_path, obstacles)
        
    if questionNum == 6:
        # Check for collisions along the path

        if np.sum(cspace) == 0:
            cspace = C2_func(robot,cspace,obstacles,q_grid)
        
        distances = C3_func(robot,cspace,q_grid,q_goal)
        path = C4_func(distances, q_grid, q_start)
        
        q_path = C5_func(q_grid, q_start, q_goal, path)

        num_collisions = C6_func(robot, q_path, obstacles)
        print("Number of collisions: ", num_collisions)
    

    if questionNum == 7:
        # Pad the configuration space and check for collisions along the path

        if np.sum(cspace) == 0:
            cspace = C2_func(robot,cspace,obstacles,q_grid)
        
        padded_cspace = C7_func(cspace)
        
        distances = C3_func(robot,padded_cspace,q_grid,q_goal)
        path = C4_func(distances, q_grid, q_start)
        
        q_path = C5_func(q_grid, q_start, q_goal, path)

        num_collisions = C6_func(robot, q_path, obstacles)
        print("Number of collisions: ", num_collisions)
        assert num_collisions == 0, "Incorrect answer, number of collisions should be 0."

        plot_path_visualization(robot, q_path, obstacles)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Enter question number')
    parser.add_argument('-q','--questionNum', type=int, help='Enter question number')
    parser.add_argument('-c','--cspace', type=str, help='Enter cspace file name', default=None)
    args = parser.parse_args()

    if args.questionNum == None:
        raise ValueError('Error: please enter a question number as a parameter')
    if args.questionNum < 1 or args.questionNum > 7:
        raise ValueError('Error: please enter a valid question number as a parameter (1-7)')

    try: 
        cspace = np.load(args.cspace)
        _ = np.sum(cspace)
        hw3_cspace(args.questionNum,cspace)
    except:
        hw3_cspace(args.questionNum)