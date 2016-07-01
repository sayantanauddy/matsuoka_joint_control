#!/usr/bin/env python

###############################################################################
# File: darwin_walk_demo.py
# Description: Executes walk_limit number of walks for the Darwin OP in gazebo
#              and records the distance travelled and average height for each
#              walk. The walking algorithm is the default one provided.
#              Also plots and creates a data file of the joint angles
# Execution: rosrun robot_walking darwin_walk_demo.py
###############################################################################

import rospy
import math
import time
import matplotlib.pyplot as plt
import numpy as np
import pylab as pyl
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


class Darwin_Walk_Demo():

    
    def __init__(self):
    
        self.model_name = 'darwin'
        self.relative_entity_name = 'world'
        
        # List to store the average model z coordinate
        self.model_avg_z_list = []
        
        # List to store the total distance covered during the walk
        self.model_distance_list = []
        
        # List to store the polled z coordinates during a walk
        self.model_polled_z = []
        
        rospy.init_node('darwin_walk_demo', anonymous=False)
        #rospy.rate(100)

        # Joints to plot
        self.joints_to_plot = ['j_pelvis_l','j_thigh1_l','j_thigh2_l','j_tibia_l','j_ankle1_l','j_ankle2_l',
                               'j_pelvis_r','j_thigh1_r','j_thigh2_r','j_tibia_r','j_ankle1_r','j_ankle2_r']
                
        # Start time
        self.t0 = time.time()
        
        # Subscribe to the /gazebo/model_states topic
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.subscriber_callback_modelstate)
        
        # Register the client for service gazebo/get_model_state
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Register the client for service gazebo/set_model_state
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        

        # Register the client for service /darwin/controller_manager/list_controllers
        rospy.wait_for_service('/darwin/controller_manager/list_controllers')
        get_joint_state = rospy.ServiceProxy('/darwin/controller_manager/list_controllers', ListControllers)

        # Service call to get joint states
        try:
             response = get_joint_state()
             controllers = response.controller
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        # Initialize map to store joint positions over time
        self.joint_plots = {}
        for controller in controllers:
            # Remove '_position_controller' to obtain joint name
            jointname = controller.name.replace('_position_controller', '')
            self.joint_plots[jointname] = []
        
        # Subscribe to the /darwin/joint_states topic
        rospy.Subscriber("/darwin/joint_states", JointState, self.subscriber_callback_jointstate)        
          

        # Create a file for joint data
        self.plot_file = open('/home/sayantan/Knowledge/Independant Studies/Robot Walking/code/robot_walking/data/darwin_walk_demo_joint_angles.csv', 'w')

        # Write the header of the data file
        writestr = "time,"
        for joint in self.joints_to_plot:
            writestr += str(joint) + ","
        
        # Remove the last comma
        writestr = writestr[:len(writestr)-1]
        writestr += "\n"
        self.plot_file.write(writestr)
        
        self.start_run()
        

    def subscriber_callback_jointstate(self, jointstate):
        # Get the current time
        curr_t = time.time()
        
        #Calculate offset from t=0
        time_diff = curr_t - self.t0
        
        # Log the joint positions
        writestr = str(round(time_diff,3)) + ","
        for jointname in self.joints_to_plot:
            writestr += str(round(jointstate.position[jointstate.name.index(jointname)],4)) + ","
            
        # Remove the last comma
        writestr = writestr[:len(writestr)-1]
            
        writestr += "\n"
        self.plot_file.write(writestr)
        
        
    def subscriber_callback_modelstate(self, dummymodelstate):
        try:
            modelstate = self.get_model_state(self.model_name,self.relative_entity_name)
            
            # Retrieve model x,y and z coordinates
            self.current_model_x = modelstate.pose.position.x
            self.current_model_y = modelstate.pose.position.y
            self.current_model_z = modelstate.pose.position.z
            
            # Store the current z coordinate in the list
            self.model_polled_z.append(self.current_model_z)
              
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def start_run(self):
        # How many times to walk
        walk_limit = 2;
        walk_counter = 0;
        # How many seconds to walk each time
        walk_time = 60;
        
        self.reset_simulation()
        
        # For each walk
        while(walk_counter < walk_limit):
            
            rospy.loginfo("Starting walk number " + str(walk_counter))
            # Walk for 20 seconds
            self.initiate_walk(walk_time, [1,0,0])
            
            rospy.loginfo("Starting evaluation for walk number " + str(walk_counter))
            # Perform evaluation using the logged data for the walk
            self.evaluate_walk()
            
            rospy.loginfo("Resetting simulation")
            self.reset_simulation()
            
            walk_counter = walk_counter + 1
        
        rospy.loginfo("Printing results")
        
        # Print results
        self.report()
        
        
        
    def initiate_walk(self, walk_seconds,walk_velocity):
        darwin = Darwin()
        darwin.set_walk_velocity(walk_velocity[0],walk_velocity[1],walk_velocity[2])
        rospy.sleep(walk_seconds)
        darwin.set_walk_velocity(0,0,0)
        
    
    def evaluate_walk(self):
    
        # Evaluate the average of self.model_polled_z
        average_z = sum(self.model_polled_z) / float(len(self.model_polled_z))
        
        # Append this value to the self.model_avg_z_list
        self.model_avg_z_list.append(average_z)
        
        # Calculate the distance travelled by the robot
        distance = math.sqrt((self.current_model_x)*(self.current_model_x) + (self.current_model_y)*(self.current_model_y))
        
        # Append the distance to the list self.model_distance_list
        self.model_distance_list.append(distance)
    
    
    def reset_simulation(self):
        # Empty the self.model_polled_z list
        self.model_polled_z = []
        
        # Send model to x=0,y=0
        model_name = 'darwin'
        pose = Pose()
        pose.position.z = 0.31
        twist = Twist()
        reference_frame = 'world'
        
        md_state = ModelState()
        md_state.model_name = model_name
        md_state.pose = pose
        md_state.twist = twist
        md_state.reference_frame = reference_frame

        # Service call to reset model
        try:
            response = self.set_model_state(md_state)             
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        rospy.loginfo("Result of model reset: " + str(response))
        
        # Reset the joints - Not implemented for now
        
        
    def report(self):
        rospy.loginfo("Average height:")
        rospy.loginfo(self.model_avg_z_list)
        rospy.loginfo("Distance travelled:")
        rospy.loginfo(self.model_distance_list)


        
if __name__ == '__main__':
    try:
        Darwin_Walk_Demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
        
        
