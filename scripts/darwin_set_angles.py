#!/usr/bin/env python

###############################################################################
# File: darwin_set_angles.py
# Description: Rotates the arms according to a sine equation
# Execution: rosrun robot_walking darwin_set_angles.py
###############################################################################

import rospy
import math
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
        self.shoulder_limits = 1.74533
        
        self.darwin = Darwin()
        self.rotate_sinusoid()
        

        
    def rotate_sinusoid(self):
    
        # Set the shoulder roll angle and elbow so that arms are at the side
        new_angles = {}
        new_angles['j_high_arm_l'] = 1.50
        new_angles['j_high_arm_r'] = 1.50
        new_angles['j_low_arm_l'] = 0.0
        new_angles['j_low_arm_r'] = 0.0
        self.darwin.set_angles_slow(new_angles)
        
        t=0
        
        while(t<120):
        
            new_angles['j_shoulder_l'] = self.shoulder_limits*(math.sin(self.omega*t))
            
            # According to right hand rule, positive angle for l shoulder=negative angle for right
            new_angles['j_shoulder_r'] = new_angles['j_shoulder_l']
            self.darwin.set_angles_slow(new_angles)
            t=t+1
        
        

if __name__ == '__main__':

    Darwin_Set_Angles()
        
