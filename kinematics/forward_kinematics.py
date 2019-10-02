'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import numpy as np
from numpy.matlib import matrix, identity
from math import sin
from math import cos
from math import sqrt
from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
	# taken from http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
			'LArm': ['LShoulderPitch', 'LShoulderRoll' , 'LElbowYaw' , 'LElbowRoll'],
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RAnkleRoll'],
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'LAnkleRoll'],
'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']    	
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)


    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
	
        sinus = sin(joint_angle)
        cosinus = cos(joint_angle)
	# taken from http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html	
	# arbitrary modified
	distances = {
        'HeadYaw': [0.0, 0.0, 100],
        'HeadPitch': [0.0, 10, 0.0],
        'LShoulderPitch': [0.0, 60, 70],
        'LShoulderRoll': [0.0, 0.0, 0.0],
        'LElbowYaw': [50, 20, 0.0],
        'LElbowRoll': [0.0, 0.0, 0.0],
        'RShoulderPitch': [0.0, -60, 40],
        'RShoulderRoll': [0.0, 2.0, 0.0],
        'RElbowYaw': [80, -20, 0.0],
        'RElbowRoll': [0.0, 0.0, 0.0],
        'LHipYawPitch': [0.0, 50.0, -85.0],
        'LHipRoll': [0.0, 10, 0.0],
        'LHipPitch': [15, 0.0, 0.0],
	'LKneePitch': [0.0, 0.0, -60],
        'LAnklePitch': [0.0, 0.0, -30],
        'LAnkleRoll': [0.0, 30, 0.0],
        'RHipYawPitch': [0.0, -30, 0.0],
        'RHipRoll': [1.0, 0.0, 0.0],
        'RHipPitch': [0.0, -10, 0.0],
        'RKneePitch': [0.0, 0.0, -50],
        'RAnklePitch': [0.0, 0.0, -70],
        'RAnkleRoll': [0.0, 0.0, 0.0]}
	# joints taken from from http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
	joints = {
        'HeadYaw': 'Z',
        'HeadPitch': 'Y',
        'RShoulderPitch': 'Y',
        'RShoulderRoll': 'Z',
        'RElbowYaw': 'X',
        'RElbowRoll': 'Z',
        'LShoulderPitch': 'Y',
        'LShoulderRoll': 'Z',
        'LElbowYaw': 'X',
        'LElbowRoll': 'Z',
        'LHipYawPitch': 'Y-Z',
        'RHipYawPitch': 'Y-Z',
        'LHipRoll': 'X',
        'LHipPitch': 'Y',
        'LKneePitch': 'Y',
        'LAnklePitch': 'Y',
        'LAnkleRoll': 'X',
        'RHipRoll': 'X',
        'RHipPitch': 'Y',
        'RKneePitch': 'Y',
        'RAnklePitch': 'Y',
        'RAnkleRoll': 'X',
        } 

	allPredifinedMatrices = { 
	'X':np.array([
                [1.0, 0.0, 0.0],
                [0.0, cosinus, -sinus],
                [0.0, sinus, cosinus]
            ]),
	'Y': np.array([
                [cosinus, 0.0, sinus],
                [0.0, 1.0, 0.0],
                [-sinus, 0.0, cosinus]
            ]),
	'Z': np.array([
                [cosinus, -sinus, 0.0],
                [sinus, cosinus, 0.0],
                [0.0, 0.0, 1.0]
            ]),
	'Y-Z': np.array([
                [cosinus,               -(sinus / sqrt(2)),          (sinus / sqrt(2))],
                [(sinus / sqrt(2)),      (1.0/2.0)*(1 + cosinus),    (1.0/2.0)*(1 - cosinus)],
                [-(sinus / sqrt(2)),     (1.0/2.0)*(1 - cosinus),    (1.0/2.0)*(1 + cosinus)]])
	}
	
	
	currentJoint = joints[joint_name]
	T[0, 3]= distances[joint_name][0]
        T[1, 3] = distances[joint_name][1]
        T[2, 3] = distances[joint_name][2]
	T[0:3, 0:3] = allPredifinedMatrices[currentJoint]
        
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
		print joint
		T *= Tl
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
