#!/usr/bin/env python

###############################################################################
# File: matsuoka_complete_network.py
# Description: Generates plots for 12 coupled matsuoka neural oscillators
#              Also executes walking motion for manually set weights and params
# Execution: python matsuoka_4_neuron_model.py
###############################################################################

import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time
# Import messages
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist
# Import services
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from controller_manager_msgs.srv import ListControllers
# Import models
from darwin_gazebo.darwin import Darwin


# Function to calculate next states
def matsuoka(state):

    global s1, s2, s3

    x1 = state[0]
    x2 = state[1]
    x3 = state[2]
    x4 = state[3]
    x5 = state[4]
    x6 = state[5]
    x7 = state[6]
    x8 = state[7]
    x9 = state[8]
    x10 = state[9]
    x11 = state[10]
    x12 = state[11]

    f1 = state[12]
    f2 = state[13]
    f3 = state[14]
    f4 = state[15]
    f5 = state[16]
    f6 = state[17]
    f7 = state[18]
    f8 = state[19]
    f9 = state[20]
    f10 = state[21]
    f11 = state[22]
    f12 = state[23]

    y1 = state[24]
    y2 = state[25]
    y3 = state[26]
    y4 = state[27]
    y5 = state[28]
    y6 = state[29]
    y7 = state[30]
    y8 = state[31]
    y9 = state[32]
    y10 = state[33]
    y11 = state[34]
    y12 = state[35]

    # Hip neurons
    x1_d = ((-1.0 * a1 * y2) - (a1 * y3) + (s1) - (b * f1) - (x1)) / float(Tr)
    x2_d = ((-1.0 * a1 * y1) - (a1 * y4) + (s1) - (b * f2) - (x2)) / float(Tr)
    x3_d = ((-1.0 * a1 * y1) - (a1 * y4) + (s1) - (b * f3) - (x3)) / float(Tr)
    x4_d = ((-1.0 * a1 * y2) - (a1 * y3) + (s1) - (b * f4) - (x4)) / float(Tr)

    # Knee neurons
    x5_d = ((-1.0 * a3 * y6) - (a5 * y1) + (s2) - (b * f5) - (x5)) / float(Tr)
    x6_d = ((-1.0 * a3 * y5) + (s2) - (b * f6) - (x6)) / float(Tr)
    x7_d = ((-1.0 * a3 * y8) - (a5 * y2) + (s2) - (b * f7) - (x7)) / float(Tr)
    x8_d = ((-1.0 * a3 * y7) + (s2) - (b * f8) - (x8)) / float(Tr)

    # Ankle neurons
    x9_d = ((-1.0 * a4 * y10) + (s3) - (b * f9) - (x9)) / float(Tr)
    x10_d = ((-1.0 * a4 * y9) - (a6 * y1) + (s3) - (b * f10) - (x10)) / float(Tr)
    x11_d = ((-1.0 * a4 * y12) + (s3) - (b * f11) - (x11)) / float(Tr)
    x12_d = ((-1.0 * a4 * y11) - (a6 * y2) + (s3) - (b * f12) - (x12)) / float(Tr)

    # Calculate the elements of the next state
    x1 += step * x1_d
    x2 += step * x2_d
    x3 += step * x3_d
    x4 += step * x4_d
    x5 += step * x5_d
    x6 += step * x6_d
    x7 += step * x7_d
    x8 += step * x8_d
    x9 += step * x9_d
    x10 += step * x10_d
    x11 += step * x11_d
    x12 += step * x12_d

    y1 = max(0.0, x1)
    y2 = max(0.0, x2)
    y3 = max(0.0, x3)
    y4 = max(0.0, x4)

    y5 = max(0.0, x5)
    y6 = max(0.0, x6)

    y7 = max(0.0, x7)
    y8 = max(0.0, x8)

    y9 = max(0.0, x9)
    y10 = max(0.0, x10)

    y11 = max(0.0, x11)
    y12 = max(0.0, x12)

    f1_d = (y1 - f1) / float(Ta)
    f2_d = (y2 - f2) / float(Ta)
    f3_d = (y3 - f3) / float(Ta)
    f4_d = (y4 - f4) / float(Ta)

    f5_d = (y5 - f5) / float(Ta)
    f6_d = (y6 - f6) / float(Ta)
    f7_d = (y7 - f7) / float(Ta)
    f8_d = (y8 - f8) / float(Ta)
    f9_d = (y9 - f9) / float(Ta)
    f10_d = (y10 - f10) / float(Ta)
    f11_d = (y11 - f11) / float(Ta)
    f12_d = (y12 - f12) / float(Ta)

    f1 += step * f1_d
    f2 += step * f2_d
    f3 += step * f3_d
    f4 += step * f4_d
    f5 += step * f5_d
    f6 += step * f6_d
    f7 += step * f7_d
    f8 += step * f8_d
    f9 += step * f9_d
    f10 += step * f10_d
    f11 += step * f11_d
    f12 += step * f12_d

    # Create the return state
    ret_state = [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12,
                 f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12,
                 y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12]

    return ret_state


def reset_simulation():

    global current_model_x, current_model_y, current_model_z, has_fallen

    # Reset the joints
    new_angles = {}
    new_angles['j_ankle1_l'] = 0.0
    new_angles['j_ankle1_r'] = 0.0
    new_angles['j_ankle2_l'] = 0.0
    new_angles['j_ankle2_r'] = 0.0
    new_angles['j_gripper_l'] = 0.0
    new_angles['j_gripper_r'] = 0.0
    new_angles['j_high_arm_l'] = 1.50  # Arm by the side of the body
    new_angles['j_high_arm_r'] = 1.50  # Arm by the side of the body
    new_angles['j_low_arm_l'] = 0.0
    new_angles['j_low_arm_r'] = 0.0
    new_angles['j_pan'] = 0.0
    new_angles['j_pelvis_l'] = 0.0
    new_angles['j_pelvis_r'] = 0.0
    new_angles['j_shoulder_l'] = 0.0
    new_angles['j_shoulder_r'] = 0.0
    new_angles['j_thigh1_l'] = 0.0
    new_angles['j_thigh1_r'] = 0.0
    new_angles['j_thigh2_l'] = 0.0
    new_angles['j_thigh2_r'] = 0.0
    new_angles['j_tibia_l'] = 0.0
    new_angles['j_tibia_r'] = 0.0
    new_angles['j_tilt'] = 0.0
    new_angles['j_wrist_l'] = 0.0
    new_angles['j_wrist_r'] = 0.0
    # Default delay of setting angles is 2 seconds
    darwin.set_angles_slow(new_angles)

    # Reset the robot position
    # Send model to x=0,y=0
    pose = Pose()
    pose.position.z = 0.31
    twist = Twist()

    md_state = ModelState()
    md_state.model_name = model_name
    md_state.pose = pose
    md_state.twist = twist
    md_state.reference_frame = 'world'

    # Service call to reset model
    try:
        response = set_model_state(md_state)
    except rospy.ServiceException, e:
        print "Darwin model state initialization service call failed: %s" % e

    rospy.loginfo("Result of model reset: " + str(response))

    # Reset the distance variable
    distance_walked = 0.0

    # Reset the position variables
    current_model_x = 0.0
    current_model_y = 0.0
    current_model_z = 0.0

    # Reset the robot fall detection flag
    has_fallen = False


# Function to retrieve the current model state - the position and orientation of the robot
def subscriber_callback_modelstate(dummymodelstate):

    global current_model_x, current_model_y, current_model_z, has_fallen

    try:
        modelstate = get_model_state(model_name, 'world')

        # Retrieve model x,y and z coordinates
        current_model_x = modelstate.pose.position.x
        current_model_y = modelstate.pose.position.y
        current_model_z = modelstate.pose.position.z

        # Check if the robot has fallen
        if (current_model_z < 0.2):
            has_fallen = True

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

has_fallen = False
current_model_x = 0.0
current_model_y = 0.0
current_model_z = 0.0

model_name = 'darwin'
rospy.init_node('darwin_walk_demo', anonymous=False)

# Subscribe to the /gazebo/model_states topic
rospy.Subscriber("/gazebo/model_states", ModelStates, subscriber_callback_modelstate)

# Register the client for service gazebo/get_model_state
rospy.wait_for_service('/gazebo/get_model_state')
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Register the client for service gazebo/set_model_state
rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

# Register the client for service /darwin/controller_manager/list_controllers
rospy.wait_for_service('/darwin/controller_manager/list_controllers')
get_joint_state = rospy.ServiceProxy('/darwin/controller_manager/list_controllers', ListControllers)

darwin = Darwin()
# Set the shoulder roll angle and elbow so that arms are at the side
new_angles = {}
new_angles['j_high_arm_l'] = 1.50
new_angles['j_high_arm_r'] = 1.50
new_angles['j_low_arm_l'] = 0.0
new_angles['j_low_arm_r'] = 0.0
darwin.set_angles_slow(new_angles)

# Declare lists to be used for plots
hip_left = []
knee_left = []
ankle_left = []
hip_right = []
knee_right = []
ankle_right = []
t_list = []

# Tunable parameters, static for now
a1 = 1.5
a2 = 1.5
a3 = 1.5
a4 = 1.5
a5 = 1.5
a6 = 1.5

s1 = 2.0
s2 = 2.0
s3 = 2.0
b = 5.0
Tr = 0.7
Ta = 0.7

# Iteration constants
step = 0.2
count = 500


# Set initial state
state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # x1-12
         0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,  # f1-12
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # y1-12

i = 0
reset_simulation()
while (i < count):
    state = matsuoka(state)

    # Unpack the state vector
    y1 = state[24]
    y2 = state[25]
    y3 = state[26]
    y4 = state[27]
    y5 = state[28]
    y6 = state[29]
    y7 = state[30]
    y8 = state[31]
    y9 = state[32]
    y10 = state[33]
    y11 = state[34]
    y12 = state[35]

    hip_left.append(y1 - y3)
    hip_right.append(y2 - y4)

    knee_left.append(y5 - y6)
    knee_right.append(y7 - y8)

    ankle_left.append(y9 - y10)
    ankle_right.append(y11 - y12)

    if (i * step > 40):
        new_angles['j_thigh2_l'] = -(y1 - y3)
        new_angles['j_thigh2_r'] = (y2 - y4)

        knee_r = y5 - y6
        knee_l = -(y7 - y8)

        if (knee_l > 0.0): knee_l = 0.0
        new_angles['j_tibia_l'] = knee_l

        if (knee_r<0.0): knee_r = 0.0

        new_angles['j_tibia_r'] = knee_r

        new_angles['j_ankle1_l'] = (y9 - y10)
        new_angles['j_ankle1_r'] = -(y11 - y12)


        darwin.set_angles_slow(new_angles, step)

    #if(has_fallen):
    #    reset_simulation()

    t_list.append(i * step)
    i += 1

# Plot
plt.figure(1)
plt.subplot(611)
plt.plot(t_list, hip_left, label="hip_left")
plt.subplot(612)
plt.plot(t_list, hip_right, label="hip_right")
plt.subplot(613)
plt.plot(t_list, knee_left, label="knee_left")
plt.subplot(614)
plt.plot(t_list, knee_right, label="knee_right")
plt.subplot(615)
plt.plot(t_list, ankle_left, label="ankle_left")
plt.subplot(616)
plt.plot(t_list, ankle_right, label="ankle_right")
plt.legend()
plt.show()

