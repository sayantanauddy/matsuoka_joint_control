#!/usr/bin/env python

###############################################################################
# File: darwin_set_angles.py
# Description: Rotates the arms according to a matsuoka oscillator
# Execution: rosrun robot_walking darwin_set_angles_matsuoka.py
###############################################################################

import rospy
import math
import matplotlib.pyplot as plt
# Import messages
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist
# Import services
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState

# Import models
from darwin_gazebo.darwin import Darwin



class Darwin_Set_Angles():

    def __init__(self):
        
        rospy.init_node('darwin_set_angles', anonymous=False)
        
        self.omega = 1.0
        self.shoulder_limit_max = 1.745
        self.shoulder_limit_min = -1.745        
        
        # Declare lists to be used for plots
        self.y_list = []
        self.t_list = []

        # Iteration constants
        self.step = 0.2
        self.count = 120
        
        # Origin of oscillation
        self.origin_j_shoulder_l = 0.0
                
        self.darwin = Darwin()
        self.rotate_sinusoid()
        
        
    # Function to calculate next states
    def matsuoka(self, state):
    
        # Tunable parameters, static for now
        a  = 2.5
        s  = 1.0
        b  = 2.0
        Tr = 0.75
        Ta = 0.75

        x1 = state[0]
        x2 = state[1]
        f1 = state[2]
        f2 = state[3]
        y1 = state[4]
        y2 = state[5]
        
            
        x1_d = ((-1.0*a*y2) + (s) - (b*f1) - (x1))/float(Tr)
        x2_d = ((-1.0*a*y1) + (s) - (b*f2) - (x2))/float(Tr)    
        
        x1 = x1 + self.step*x1_d
        x2 = x2 + self.step*x2_d
        
        y1 = max(0.0, x1)
        y2 = max(0.0, x2)
        
        
        f1_d = (y1 - f1)/float(Ta)
        f2_d = (y2 - f2)/float(Ta)
        
        f1 = f1 + self.step*f1_d
        f2 = f2 + self.step*f2_d
        
        return [x1, x2, f1, f2, y1, y2]
        
            
    def rotate_sinusoid(self):
    
        # Set initial state
        self.state = [0.0, 1.0, 0.0, 1.0, 0.0 ,0.0]
    
        # Set the shoulder roll angle and elbow so that arms are at the side
        new_angles = {}
        new_angles['j_high_arm_l'] = 1.50
        new_angles['j_high_arm_r'] = 1.50
        new_angles['j_low_arm_l'] = 0.0
        new_angles['j_low_arm_r'] = 0.0
        self.darwin.set_angles_slow(new_angles)
        
        i=0
        while(i<self.count):
            self.state = self.matsuoka(self.state)
            y_oscillator = self.state[4]-self.state[5]
            y_joint = y_oscillator + self.origin_j_shoulder_l

            self.t_list.append(i*self.step)
            
            # Check angle limits
            if (y_joint > self.shoulder_limit_max):
                y_joint = self.shoulder_limit_max
              
            if (y_joint < self.shoulder_limit_min):
                y_joint = self.shoulder_limit_min
 
            self.y_list.append(y_joint)
                        
            new_angles['j_shoulder_l'] = y_joint
            
            # According to right hand rule, positive angle for l shoulder=negative angle for right
            new_angles['j_shoulder_r'] = new_angles['j_shoulder_l']
            
            # The last parameter is the delay, this should match the step size
            self.darwin.set_angles_slow(new_angles,self.step)
            
            print i
            i=i+1

        # Plot
        plt.figure()
        plt.plot(self.t_list,self.y_list)
        plt.show()
                

if __name__ == '__main__':

    Darwin_Set_Angles()
        
