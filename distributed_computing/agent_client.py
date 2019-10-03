'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib

import os
import sys
from numpy.matlib import identity
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import *
import threading
from keyframes import leftBackToStand

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes(keyframes), args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform(effector_name, transform), args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.client = xmlrpclib.ServerProxy("http://localhost:4200", allow_none=True)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.client.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.client.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.client.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.client.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.client.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.client.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print "Setting angle\n"
    agent.set_angle('HeadYaw', 30.0)
    print "Getting angle\n"
    agent.get_angle('HeadYaw')
    print "Getting posture\n"
    agent.get_posture()
    print "Getting transform\n"
    agent.get_transform('HeadYaw')
    print "Executing keyframes (blocking) \n"
    agent.execute_keyframes(leftBackToStand())
    postHandler = PostHandler(agent)
    print "Executing keyframes (non-blocking) \n"
    postHandler.execute_keyframes(leftBackToStand())


