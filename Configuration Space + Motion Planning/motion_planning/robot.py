# import numpy as np
# from scipy.spatial.transform import Rotation
import pybullet as p
import numpy as np
import time

#finds the urdf
## TODO make this more general
import os
robot_parent_path = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(robot_parent_path,"robot/simple.urdf")

# def wrap_to_pi(data):
#     '''
#     wraps an array type set of data from -pi tp pi. expects a numpy array or list
#     '''
#     for i in range(len(data)):
#         data[i] = ((data[i] + np.pi)%np.pi - np.pi)
#     return data

#TODO resetJointState can violate the joint limits. Do we want to allow this or correct for it?

class Simple_Manipulator():
    def __init__(self, base_position = None, base_orientation = None):
        
        self.urdf = urdf_path

        if base_position is None:
            self.base_position = [0,0,0]
        else:
            self.base_position = base_position
        if base_orientation is None:
            self.base_orientation = p.getQuaternionFromEuler([0,0,0])
        else:
            self.base_orientation = base_orientation
        #load robot model - requires p.connect to have been called
        self.id = p.loadURDF(self.urdf, self.base_position, self.base_orientation, useFixedBase = True)

        self.actuated_joint_ids, self.actuated_joint_names, self.lower_lims, self.upper_lims, self.ee_link_id = self._get_actuated_joints()
        self.num_actuated_joints = len(self.actuated_joint_ids)

    def _get_actuated_joints(self):
        joint_ids = []
        joint_names = []
        joint_lower_lims = []
        joint_upper_lims = []
        #print("Joint Info")
        for i in range(p.getNumJoints(self.id)):
            joint_info = p.getJointInfo(self.id, i)
            #print(joint_info)
            if joint_info[2] == 0: #revolute joint
                joint_ids.append(i)
                joint_names.append(str(joint_info[1]))
                joint_lower_lims.append(joint_info[8])
                joint_upper_lims.append(joint_info[9])
            #check joint info for ee link
            if b'end_effector' in joint_info:
                ee_link = i
                # print("found ee link : {}".format(i))
        return joint_ids, joint_names, joint_lower_lims, joint_upper_lims, ee_link

    def get_joint_positions(self):
        states = p.getJointStates(self.id, self.actuated_joint_ids)
        positions = [s[0] for s in states]
        return positions
    
    def set_joint_positions(self, joint_positions):
        '''
        '''
        assert len(joint_positions) == self.num_actuated_joints
        for i, j in zip(self.actuated_joint_ids ,joint_positions):
            p.resetJointState(self.id,i,j)

    def move_to(self, joint_positions, total_time = 1.0, t_step = 0.05, max_speed = None):
        ''' Visualizes Movement from one set of joint positions to another. 
        Interpolates linear motion through joint space. Does not consider collisions
        Inputs:
            joint_positions     numpy array of joint positions with len = n_joints
            total_time          total elapsed time for motion
            t_step              float time step wait between each visualized point
            speed               maximum radians per second of joint movement (NOT IMPLEMENTED)'''
        try:
            joint_positions = joint_positions.reshape(-1)
        except AttributeError:
            joint_positions = np.array(joint_positions)
        assert len(joint_positions) == self.num_actuated_joints, "commanded joints {} must have {} elements".format(len(joint_positions), self.num_actuated_joints)

        current_joint_pos = np.array(self.get_joint_positions())

        #calculate maximum joint delta
        max_delta = np.max(np.abs(current_joint_pos))
        if not max_speed is None:
            raise(NotImplementedError("Max speed is not implementd"))
        
        #interpolation
        n_steps = int(total_time/t_step)
        intermediate_steps = np.linspace(current_joint_pos, joint_positions, n_steps)

        #command joints
        for joints in intermediate_steps:
            self.set_joint_positions(joints)
            time.sleep(t_step)

    def forward_kinematics(self, joint_positions = None):
        '''
        '''
        start_joint_positions = None
        if not joint_positions is None:
            assert len(joint_positions) == self.num_actuated_joints
            start_joint_positions = self.get_joint_positions()
            #temporarily move arm
            self.set_joint_positions(joint_positions)
        
        #get all collisions with this robot
        link_info = p.getLinkState(self.id, self.ee_link_id)
        ee_position = link_info[0]
        ee_orientation = link_info[1]

        #return robot state to original
        if not start_joint_positions is None:
            self.set_joint_positions(start_joint_positions)
            p.performCollisionDetection()
        
        return ee_position, ee_orientation

    def inverse_kinematics(self, ee_position, ee_orientation = None):
        '''
        Note, will still return joint positions even if ee_position is unreachable
        '''
        joint_positions = p.calculateInverseKinematics(self.id,self.ee_link_id,ee_position, ee_orientation)
        return joint_positions

    def _collision_check(self):
        '''Checks for collisions at current position'''

        #get all collisions with this robot
        p.performCollisionDetection()
        collisions = p.getContactPoints(self.id)
        return not len(collisions) == 0

    def is_in_collision(self, joint_positions = None):
        '''
        if joint_positions is None, checks for a collision in the current configuration 
        '''

        start_joint_positions = None
        if not joint_positions is None:
            self.assert_joints(joint_positions)
            start_joint_positions = self.get_joint_positions()
            #temporarily move arm
            self.set_joint_positions(joint_positions)
        
        collision_result = self._collision_check()

        #return robot state to original
        if not start_joint_positions is None:
            self.set_joint_positions(start_joint_positions)
            p.performCollisionDetection()

        return collision_result
    
    def is_joints(self, joint_positions, oneD = False):
        '''returns true if valid joint format'''
        if (type(joint_positions) == list):
            return len(joint_positions) == self.num_actuated_joints
        elif (type(joint_positions) == np.ndarray):
            if oneD:
                return (len(joint_positions.shape) == 1) and (len(joint_positions == self.num_actuated_joints))
            else:
                return joint_positions.shape[-1] == self.num_actuated_joints
        else:
            return False
        
    def assert_joints(self, joint_positions, oneD = False):
        '''same as is joints but with assertions if valid joint format'''
        if (type(joint_positions) == list):
            assert len(joint_positions) == self.num_actuated_joints, "Joints {} must have len {}".format(joint_positions, self.num_actuated_joints)
        elif (type(joint_positions) == np.ndarray):
            if oneD:
                assert (len(joint_positions.shape) == 1) and (len(joint_positions == self.num_actuated_joints), "Joints {} must have shape {}".format(joint_positions, (self.num_actuated_joints)))
            else:
                assert joint_positions.shape[-1] == self.num_actuated_joints, "joints[-1] is not {}".format(self.num_actuated_joints)
        else:
            raise ValueError("joint positions {} is type {} but must be {}".format(joint_positions, 
                                                                                   type(joint_positions),
                                                                                   (list, np.ndarray)))

    def check_edge(self, start_joint_positions, end_joint_positions, resolution = 10):
        '''Checks allong a joint trajectory, interpolating in joint space, for collisions.
        Returns True, if edge is collision free, False, if edge is in collision'''
        
        self.assert_joints(start_joint_positions)
        self.assert_joints(end_joint_positions)
        # assert self.is_joints(start_joint_positions), "{} are not valid joint positions".format(start_joint_positions)
        # assert self.is_joints(end_joint_positions), "{} are not valid joint positions".format(end_joint_positions)
        
        #interpolate joints
        interp_joints = np.linspace(start_joint_positions, end_joint_positions, resolution)
        
        #check all interpolated positions for collisions
        edge_in_collision = False
        init_joints = self.get_joint_positions() #put the robot back the way it was when done
        for joints in interp_joints:
            self.set_joint_positions(joints)
            if self._collision_check():
                edge_in_collision = True
                break
        self.set_joint_positions(init_joints)
        
        return not edge_in_collision


if __name__ == "__main__":

    #start pybullet
    physicsClient = p.connect(p.GUI)
    # time.sleep(0.1)
    p.setGravity(0,0,-10)

    #init robot
    robot = Simple_Manipulator()

    #add obstacle 
    cs = p.createCollisionShape(p.GEOM_SPHERE, radius=0.3)
    vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.3)
    body_start_pos = [0.5,0.5,0.5]
    body_start_orientation = p.getQuaternionFromEuler([0,0,0])
    p.createMultiBody(0,cs,vs,body_start_pos, body_start_orientation)
    
    
    print("\n\n\n\nStarting Demo...")

    #check start fk
    current_position, _ = robot.forward_kinematics()
    print("Robot current end effector position = {}".format(current_position))
    print()
    # #pause briefly
    # time.sleep(2)

    #set robot position
    pos = [0.1, 0.1, 0.1, 0.1]
    print("commanding joint positions {}".format(pos))
    _ = input("Hit Enter to Continue")
    robot.set_joint_positions(pos)
    time.sleep(1)
    print()

    # #check fk
    # print()
    # current_position, _ = robot.forward_kinematics()
    # print("Robot end effector position = {}".format(current_position))
    # _ = input("Hit Enter to Continue")
    # print()

    #check fk for positions outside joint lims
    ## pb automatically wraps the joint lims
    pos = [100, 100, 100, 100]
    print("checking fk for {}".format(pos))
    ee_psoition, _ = robot.forward_kinematics(pos)
    print("ee_position for joints {} is {}".format(pos, ee_psoition))
    _ = input("Hit Enter to Continue")


    print()
    #set robot position
    # pos = [0.1, 0.1, 0.1, 0.1]
    print("commanding joint positions {}".format(pos))
    _ = input("Hit Enter to Continue")
    robot.set_joint_positions(pos)
    time.sleep(1)
    print()

    #check collisions
    print("Checking robot current collision state")
    in_collision = robot.is_in_collision()
    print("Robot is in collision = {}".format(in_collision))
    _ = input("Hit Enter to Continue")
    print()

    #check collisions 2
    print("Checking robot collision state for joints {}".format(pos))
    in_collision = robot.is_in_collision(pos)
    print("Robot is in collision = {}".format(in_collision))
    _ = input("Hit Enter to Continue")
    print()

    #check ik
    print("calculating ik solution for {}".format(body_start_pos))
    joint_soln = robot.inverse_kinematics(body_start_pos)
    print("IK soln = {}".format(joint_soln))

    #command robot into collision
    print("moving to positions {}".format(joint_soln))
    robot.set_joint_positions(joint_soln)
    print("joint positions = {}".format(robot.get_joint_positions()))
    _ = input("Hit Enter to Continue")
    print()

    #check collisions
    print("Checking robot collision state")
    in_collision = robot.is_in_collision()
    print("Robot is in collision = {}".format(in_collision))
    _ = input("Hit Enter to End Demo")
    # print()

    steps = 1000
    for i in range(steps):
        p.stepSimulation()
        time.sleep(1./240)

    #clean up
    p.disconnect()

