#!/usr/bin/env python

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


class DarwinWalkOptimization():

    def __init__(self):

        ################################## Constant Definitions ##################################

        # Matsuoka Oscillator related initialization

        self.WEIGHT_BOUNDS = [0.5, 7.5]  # Weights of the neural oscillator connections
        self.Tr1_BOUNDS = [0.25, 1.5]  # Rise time constant
        self.Tr2_BOUNDS = [0.25, 0.75]  # Rise time constant
        self.Ta1_BOUNDS = [0.25, 1.5]  # Adaptation time constant
        self.Ta2_BOUNDS = [0.25, 0.75]  # Adaptation time constant
        self.b_BOUNDS = [1.0, 8.0]  # Constant of the Matsuoka Oscillator
        self.s_BOUNDS = [1.0, 3.0]  # Constant of the Matsuoka Oscillator
        self.TRIAL_DURATION = 30  # How many seconds should each trial last
        self.STEP_DURATION = 0.2  # The step increment in seconds

        # PSO specific initialization

        self.POPULATION_SIZE = 50
        self.MAX_ITERS = 100
        self.VEL_BOUND_PCT = 0.2  # Velocity is at most 20% of the position range
        self.C1 = 2.0  # Acceleration constant
        self.C2 = 2.0  # Acceleration constant
        # Inertial weight, linearly scales from 0.9 to 0.4 over the entire run
        self.INERTIAL_WEIGHT_BOUNDS = [0.4, 0.9]

        # Darwin specific initialization

        # Hashmap of joint origins
        self.joint_origins = {}
        self.joint_origins['j_thigh1_l'] = -0.524
        self.joint_origins['j_thigh2_l'] = 0.611
        self.joint_origins['j_tibia_l'] = -1.134
        self.joint_origins['j_ankle1_l'] = 0.0
        self.joint_origins['j_thigh1_r'] = 0.524
        self.joint_origins['j_thigh2_r'] = -0.611
        self.joint_origins['j_tibia_r'] = 1.134
        self.joint_origins['j_ankle1_r'] = 0.0

        # Hashmap of joint upper limits
        self.joint_upper_limits = {}
        self.joint_upper_limits['j_thigh1_l'] = 0.0
        self.joint_upper_limits['j_thigh2_l'] = 1.745
        self.joint_upper_limits['j_tibia_l'] = 0.0
        self.joint_upper_limits['j_ankle1_l'] = 1.047
        self.joint_upper_limits['j_thigh1_l'] = 0.0
        self.joint_upper_limits['j_thigh2_r'] = 0.524
        self.joint_upper_limits['j_tibia_r'] = 2.269
        self.joint_upper_limits['j_ankle1_r'] = 1.047

        # Hashmap of joint lower limits
        self.joint_lower_limits = {}
        self.joint_lower_limits['j_thigh2_l'] = -0.524
        self.joint_lower_limits['j_tibia_l'] = -2.269
        self.joint_lower_limits['j_ankle1_l'] = -1.047
        self.joint_lower_limits['j_thigh2_r'] = -1.745
        self.joint_lower_limits['j_tibia_r'] = 0.0
        self.joint_lower_limits['j_ankle1_r'] = -1.047

        # Variable to track distance walked by each individual
        self.distance_walked = 0.0

        # Variables to track the position of the robot
        self.current_model_x = 0.0
        self.current_model_y = 0.0
        self.current_model_z = 0.0

        # Variable (list) to track the height of the robot
        self.model_polled_z = []

        # Variable to tracj=k if the robot has fallen
        self.has_fallen = False

        # Constant fall limit - If the z coordinate of the robot is below this, then the robot has fallen
        self.DARWIN_COM_FALL_LIMIT = 0.2

        # ROS specific initialization

        self.model_name = 'darwin'
        self.relative_entity_name = 'world'

        # Initialize the ROS node
        rospy.init_node('darwin_walk_demo', anonymous=False)

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
        self.get_joint_state = rospy.ServiceProxy('/darwin/controller_manager/list_controllers', ListControllers)

        # Create the Darwin model
        self.darwin = Darwin()

        # Initialize log file

        # Current time
        curr_time = time.strftime('%Y%m%d%H%M%S')

        # Create a log file
        self.log_file = open('/home/sayantan/Knowledge/Independant Studies/Robot Walking/code/robot_walking/data/log_'
                             + str(curr_time) + '_matsuoka_walk_optimization.log', 'w')
        
        # To control if log contents are dumped on screen as well                     
        self.verbose = True
                             
        self.log_file.write('Created log file at ' + str(curr_time) + '\n')
        
        # Write the parameters to the log file
        self.log_file.write('\n')
        self.log_file.write("Constants and Parameters: "  + "\n")
        self.log_file.write("###########################################" + "\n")
        self.log_file.write("WEIGHT_BOUNDS = " + str(self.WEIGHT_BOUNDS) + "\n")
        self.log_file.write("Tr1_BOUNDS = " + str(self.Tr1_BOUNDS) + "\n")
        self.log_file.write("Tr2_BOUNDS = " + str(self.Tr2_BOUNDS) + "\n")
        self.log_file.write("Ta1_BOUNDS = " + str(self.Ta1_BOUNDS) + "\n")
        self.log_file.write("Ta2_BOUNDS = " + str(self.Ta2_BOUNDS) + "\n")
        self.log_file.write("b_BOUNDS = " + str(self.b_BOUNDS) + "\n")
        self.log_file.write("s_BOUNDS = " + str(self.s_BOUNDS) + "\n")
        self.log_file.write("TRIAL_DURATION = " + str(self.TRIAL_DURATION) + "\n")
        self.log_file.write("STEP_DURATION = " + str(self.STEP_DURATION) + "\n")
        self.log_file.write("POPULATION_SIZE = " + str(self.POPULATION_SIZE) + "\n")
        self.log_file.write("MAX_ITERS = " + str(self.MAX_ITERS) + "\n")
        self.log_file.write("VEL_BOUND_PCT = " + str(self.VEL_BOUND_PCT) + "\n")
        self.log_file.write("C1 = " + str(self.C1) + "\n")
        self.log_file.write("C2 = " + str(self.C2) + "\n")
        self.log_file.write("INERTIAL_WEIGHT_BOUNDS = " + str(self.INERTIAL_WEIGHT_BOUNDS) + "\n")
        self.log_file.write("###########################################" + "\n" + "\n")


        # Initiate the walk optimization
        self.pso()


    # Logging function
    def log_write(self, logtext):
    
        curr_time = time.strftime('%Y%m%d%H%M%S')
        logtext = "[" + str(curr_time) +"] " + logtext
        self.log_file.write(logtext + "\n")
        
        if self.verbose:
            print logtext
        
    # Function to implement the differential equations of the Matsuoka oscillator
    def matsuoka(self, state, individual):

        # Unpack the individual 
        a1 = individual[0]
        a2 = individual[1]
        a3 = individual[2]
        a4 = individual[3]
        a5 = individual[4]
        a6 = individual[5]
        Tr1 = individual[6] 
        Tr2 = individual[7]
        Ta1 = individual[8]
        Ta2 = individual[9]
        b = individual[10]
        s = individual[11]

        # Unpack the state vector
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
        x1_d = ((-1.0 * a1 * y2) - (a1 * y3) + (s) - (b * f1) - (x1)) / float(Tr1)
        x2_d = ((-1.0 * a1 * y1) - (a1 * y4) + (s) - (b * f2) - (x2)) / float(Tr1)
        x3_d = ((-1.0 * a1 * y1) - (a1 * y4) + (s) - (b * f3) - (x3)) / float(Tr1)
        x4_d = ((-1.0 * a1 * y2) - (a1 * y3) + (s) - (b * f4) - (x4)) / float(Tr1)

        # Knee neurons
        x5_d = ((-1.0 * a3 * y6) - (a5 * y1) + (s) - (b * f5) - (x5)) / float(Tr2)
        x6_d = ((-1.0 * a3 * y5) + (s) - (b * f6) - (x6)) / float(Tr2)
        x7_d = ((-1.0 * a3 * y8) - (a5 * y2) + (s) - (b * f7) - (x7)) / float(Tr2)
        x8_d = ((-1.0 * a3 * y7) + (s) - (b * f8) - (x8)) / float(Tr2)

        # Ankle neurons
        x9_d = ((-1.0 * a4 * y10) + (s) - (b * f9) - (x9)) / float(Tr2)
        x10_d = ((-1.0 * a4 * y9) - (a6 * y1) + (s) - (b * f10) - (x10)) / float(Tr2)
        x11_d = ((-1.0 * a4 * y12) + (s) - (b * f11) - (x11)) / float(Tr2)
        x12_d = ((-1.0 * a4 * y11) - (a6 * y2) + (s) - (b * f12) - (x12)) / float(Tr2)

        # Calculate the elements of the next state
        x1 += self.STEP_DURATION * x1_d
        x2 += self.STEP_DURATION * x2_d
        x3 += self.STEP_DURATION * x3_d
        x4 += self.STEP_DURATION * x4_d
        x5 += self.STEP_DURATION * x5_d
        x6 += self.STEP_DURATION * x6_d
        x7 += self.STEP_DURATION * x7_d
        x8 += self.STEP_DURATION * x8_d
        x9 += self.STEP_DURATION * x9_d
        x10 += self.STEP_DURATION * x10_d
        x11 += self.STEP_DURATION * x11_d
        x12 += self.STEP_DURATION * x12_d

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

        f1_d = (y1 - f1) / float(Ta1)
        f2_d = (y2 - f2) / float(Ta1)
        f3_d = (y3 - f3) / float(Ta1)
        f4_d = (y4 - f4) / float(Ta1)

        f5_d = (y5 - f5) / float(Ta2)
        f6_d = (y6 - f6) / float(Ta2)
        f7_d = (y7 - f7) / float(Ta2)
        f8_d = (y8 - f8) / float(Ta2)
        f9_d = (y9 - f9) / float(Ta2)
        f10_d = (y10 - f10) / float(Ta2)
        f11_d = (y11 - f11) / float(Ta2)
        f12_d = (y12 - f12) / float(Ta2)

        f1 += self.STEP_DURATION * f1_d
        f2 += self.STEP_DURATION * f2_d
        f3 += self.STEP_DURATION * f3_d
        f4 += self.STEP_DURATION * f4_d
        f5 += self.STEP_DURATION * f5_d
        f6 += self.STEP_DURATION * f6_d
        f7 += self.STEP_DURATION * f7_d
        f8 += self.STEP_DURATION * f8_d
        f9 += self.STEP_DURATION * f9_d
        f10 += self.STEP_DURATION * f10_d
        f11 += self.STEP_DURATION * f11_d
        f12 += self.STEP_DURATION * f12_d

        # Create the return state
        ret_state = [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12,
                     f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12,
                     y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12]

        return ret_state
     

    # Function for Walking using Matsuoka Oscillator
    def matsuoka_walking_fitness(self, individual):
        
        # Reset the simulation before each walk
        self.reset_simulation()

        # Set initial state - [x1-12, f1-12, y1-12]
        state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # x1-12
                 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,  # f1-12
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # y1-12
                            
        # Set the shoulder roll angle and elbow so that arms are at the side
        new_angles = {}
        new_angles['j_high_arm_l'] = 1.50
        new_angles['j_high_arm_r'] = 1.50
        new_angles['j_low_arm_l'] = 0.0
        new_angles['j_low_arm_r'] = 0.0
        new_angles['j_tilt'] = -0.262 # Tilt the head forward a little
        self.darwin.set_angles_slow(new_angles, self.STEP_DURATION)
        
        # Calculate the number of iterations
        num_iterations = self.TRIAL_DURATION / self.STEP_DURATION
        
        i = 0
        while (i < num_iterations):
            
            state = self.matsuoka(state, individual)
            
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
        
            # Calculate the joint angles
            hip_left = -(y2 - y4) # (-) sign according to right hand rotation rule
            hip_right = (y1 - y3)
            knee_left = -(y7 - y8) # (-) sign according to right hand rotation rule
            knee_right = (y5 - y6)
            ankle_left = -(y12 - y11) # (-) sign according to right hand rotation rule
            ankle_right = (y10 - y9)
            
            # Validate the angles so that they do not exceed the max and min joint limits
            # Check the lower limits
            if hip_left<self.joint_lower_limits['j_thigh2_l']: hip_left = self.joint_lower_limits['j_thigh2_l']
            if hip_right<self.joint_lower_limits['j_thigh2_r']: hip_right = self.joint_lower_limits['j_thigh2_r']
            
            if knee_left<self.joint_lower_limits['j_tibia_l']: knee_left = self.joint_lower_limits['j_tibia_l']        
            if knee_right<self.joint_lower_limits['j_tibia_r']: knee_right = self.joint_lower_limits['j_tibia_r']
            
            if ankle_left<self.joint_lower_limits['j_ankle1_l']: ankle_left = self.joint_lower_limits['j_ankle1_l']
            if ankle_right<self.joint_lower_limits['j_ankle1_r']: ankle_right = self.joint_lower_limits['j_ankle1_r']
            
            # Check the upper limits
            if hip_left>self.joint_upper_limits['j_thigh2_l']: hip_left = self.joint_upper_limits['j_thigh2_l']
            if hip_right>self.joint_upper_limits['j_thigh2_r']: hip_right = self.joint_upper_limits['j_thigh2_r']
            
            if knee_left>self.joint_upper_limits['j_tibia_l']: knee_left = self.joint_upper_limits['j_tibia_l']        
            if knee_right>self.joint_upper_limits['j_tibia_r']: knee_right = self.joint_upper_limits['j_tibia_r']
            
            if ankle_left>self.joint_upper_limits['j_ankle1_l']: ankle_left = self.joint_upper_limits['j_ankle1_l']
            if ankle_right>self.joint_upper_limits['j_ankle1_r']: ankle_right = self.joint_upper_limits['j_ankle1_r']
            
            # Wait for 10 seconds (till the oscillations stabilize) before assigning angles to joints
            if (i * self.STEP_DURATION > 10):
                # Hip
                new_angles['j_thigh2_l'] = hip_left
                new_angles['j_thigh2_r'] = hip_right
                
                # Knee
                new_angles['j_tibia_l'] = knee_left
                new_angles['j_tibia_r'] = knee_right
                
                # Ankle
                new_angles['j_ankle1_l'] = ankle_left
                new_angles['j_ankle1_r'] = ankle_right
                
                # Set the robot joint angles
                self.darwin.set_angles_slow(new_angles, self.STEP_DURATION)
                
                # Calculate the current distance travelled by the robot 
                # Start position is always (x=0, y=0)
                # self.distance_walked = np.sign(self.current_model_x) * math.sqrt((self.current_model_x)**2 + (self.current_model_y)**2)
                # Consider only the distance in x direction as fitness
                
                if (self.current_model_x == 0.0): 
                    self.distance_walked = -1.0 # Penalize individuals which do not cause any movement
                else:
                    self.distance_walked = self.current_model_x
            
                # If the robot has fallen, return the distance travelled till now as the fitness score
                if (self.has_fallen):
                    self.log_write("Robot fall detected")
                    self.log_write("Location: x = " + str(self.current_model_x) + " y = " + str(self.current_model_y) + " z = " + str(self.current_model_z))
                    self.log_write("Current fitness = " + str(self.distance_walked))
                    return self.distance_walked
            
            i += 1
            
        # If the trial duration has ended return the distance travelled as the fitness score
        # If the control has reached here, then the robot has not fallen. Hence it gets a bonus score
        self.log_write("Robot did not fall during the trial")
        self.log_write("Location: x = " + str(self.current_model_x) + " y = " + str(self.current_model_y) + " z = " + str(self.current_model_z))
        self.log_write("Current fitness = " + str(self.distance_walked))
        return self.distance_walked + 0.2*self.distance_walked
        
        
    # Function to reset the robot in the simulation (before start of run and after a fall)    
    def reset_simulation(self):

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
        new_angles['j_tilt'] = -0.262 # Tilt the head forward a little
        new_angles['j_wrist_l'] = 0.0
        new_angles['j_wrist_r'] = 0.0
        # Default delay of setting angles is 2 seconds
        self.darwin.set_angles_slow(new_angles)

        # Reset the robot position
        # Send model to x=0,y=0
        pose = Pose()
        pose.position.z = 0.31
        twist = Twist()

        md_state = ModelState()
        md_state.model_name = self.model_name
        md_state.pose = pose
        md_state.twist = twist
        md_state.reference_frame = self.relative_entity_name

        # Service call to reset model
        try:
            response = self.set_model_state(md_state)
        except rospy.ServiceException, e:
            self.log_write("Darwin model state initialization service call failed: " + str(e))

        # Reset the distance variable
        self.distance_walked = 0.0

        # Reset the position variables
        self.current_model_x = 0.0
        self.current_model_y = 0.0
        self.current_model_z = 0.0

        # Reset the height list
        self.model_polled_z = []

        # Reset the robot fall detection flag
        self.has_fallen = False
        

    # Function to retrieve the current model state - the position and orientation of the robot
    # This function also sets the fall detection flag if the robot has fallen
    def subscriber_callback_modelstate(self, dummymodelstate):
        try:
            modelstate = self.get_model_state(self.model_name, self.relative_entity_name)

            # Retrieve model x,y and z coordinates
            self.current_model_x = modelstate.pose.position.x
            self.current_model_y = modelstate.pose.position.y
            self.current_model_z = modelstate.pose.position.z

            # Store the current z coordinate in the list
            self.model_polled_z.append(self.current_model_z)

            # Check if the robot has fallen
            if (self.current_model_z < self.DARWIN_COM_FALL_LIMIT):
                self.has_fallen = True

        except rospy.ServiceException, e:
            self.log_write("Service call failed: " + str(e))
        

    # Function for Particle Swarm Optimization
    def pso(self):

        # PSO specific variables

        # The GBest vector - The first 12 elements are positions, the last element is
        # the fitness score
        self.GBest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1000.0]

        # List of individual vectors
        self.population = []

        # Initialize the population
        index = 0
        while (index < self.POPULATION_SIZE):
        
            # Generate random positions
            individual_positions = self.generate_random_individual_positions()
            
            # Generate random velocities
            individual_velocities = self.generate_random_individual_velocities()
            
            # Initialize pbest positions to the initial positions
            individual_pbest_positions = individual_positions
            
            # Initialize the pbest score to a negative number
            pbest_score = -1000.0
            
            # Create the individual vector - contains 37 elements
            individual = individual_positions + individual_velocities + individual_pbest_positions + [pbest_score]

            # Add the individual to the population
            self.population.append(individual)
            
            index += 1

            
        # Main PSO logic
        iteration = 0
        while (iteration < self.MAX_ITERS):

            self.log_write("===========================")

            # Set the linearly scaled inertial weight
            w = self.INERTIAL_WEIGHT_BOUNDS[1] - (iteration * (self.INERTIAL_WEIGHT_BOUNDS[1] - self.INERTIAL_WEIGHT_BOUNDS[0]) / self.MAX_ITERS)

            index = 0
            
            for individual in self.population:
                
                self.log_write("---------------------------")
                self.log_write("Executing iteration number: " + str(iteration) + " Individual number: " + str(index))
                
                # Calculate fitness of individual
                indiv_fitness = self.matsuoka_walking_fitness(individual)
                
                # Check pbest (last element of the individual vector)
                if (indiv_fitness > individual[36]):
                    # Set the individual's pbest as the current fitness
                    individual[36] = indiv_fitness
                    # Copy the current position into the individual's pbest position
                    individual[24:36] = individual[0:12]
                    # Set the individual in the population
                    self.population[index] = individual
                    
                # Check gbest and set if the current fitness is better
                if (indiv_fitness > self.GBest[12]):
                    self.GBest[12] = indiv_fitness
                    self.GBest[0:12] = individual[0:12]
                    
                self.log_write("Pbest = " + str(individual[36]))
                self.log_write("Gbest = " + str(self.GBest[12]))
                    
                index += 1
                
            # Once pbest and gbest have been determined for all individuals, position and velocities need to be updated
            index = 0
            for individual in self.population:

                # Unpack the vector
                pos = individual[0:12]  # 12 element vector
                vel = individual[12:24]  # 12 element vector
                pbest = individual[24:36]  # 12 element vector
                new_pos = []
                new_vel = []

                # Generate 2 random numbers
                rand1 = random.random()
                rand2 = random.random()

                # Calculate new velocities
                i = 0
                for v in vel:
                    new_v = w * v + self.C1 * rand1 * (pbest[i] - pos[i]) + self.C2 * rand2 * (self.GBest[i] - pos[i])

                    # Velocity bound checking
                    # For the network weights
                    if (i >= 0 and i < 6):
                        if (new_v < self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif (i == 6):
                        if (new_v < self.VEL_BOUND_PCT * self.Tr1_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Tr1_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Tr1_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Tr1_BOUNDS[1]
                    elif (i == 7):
                        if (new_v < self.VEL_BOUND_PCT * self.Tr2_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Tr2_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Tr2_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Tr2_BOUNDS[1]
                    elif (i == 8):
                        if (new_v < self.VEL_BOUND_PCT * self.Ta1_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Ta1_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Ta1_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Ta1_BOUNDS[1]
                    elif (i == 9):
                        if (new_v < self.VEL_BOUND_PCT * self.Ta2_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Ta2_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Ta2_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Ta2_BOUNDS[1]
                    elif (i == 10):
                        if (new_v < self.VEL_BOUND_PCT * self.b_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.b_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.b_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.b_BOUNDS[1]
                    elif (i == 11):
                        if (new_v < self.VEL_BOUND_PCT * self.s_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.s_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.s_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.s_BOUNDS[1]

                    new_vel.append(new_v)
                    i += 1
                    
                    
                # Calculate new positions
                i = 0
                for p in pos:
                    new_p = p + new_vel[i]

                    # Position bound checking
                    # For the network weights
                    if (i >= 0 and i < 6):
                        if (new_p < self.WEIGHT_BOUNDS[0]):
                            new_p = self.WEIGHT_BOUNDS[0]
                        if (new_p > self.WEIGHT_BOUNDS[1]):
                            new_p = self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif (i == 6):
                        if (new_p < self.Tr1_BOUNDS[0]):
                            new_p = self.Tr1_BOUNDS[0]
                        if (new_p > self.Tr1_BOUNDS[1]):
                            new_p = self.Tr1_BOUNDS[1]
                    elif (i == 7):
                        if (new_p < self.Tr2_BOUNDS[0]):
                            new_p = self.Tr2_BOUNDS[0]
                        if (new_p > self.Tr2_BOUNDS[1]):
                            new_p = self.Tr2_BOUNDS[1]                            
                    elif (i == 8):
                        if (new_p < self.Ta1_BOUNDS[0]):
                            new_p = self.Ta1_BOUNDS[0]
                        if (new_p > self.Ta1_BOUNDS[1]):
                            new_p = self.Ta1_BOUNDS[1]
                    elif (i == 9):
                        if (new_p < self.Ta2_BOUNDS[0]):
                            new_p = self.Ta2_BOUNDS[0]
                        if (new_p > self.Ta2_BOUNDS[1]):
                            new_p = self.Ta2_BOUNDS[1]
                    elif (i == 10):
                        if (new_p < self.b_BOUNDS[0]):
                            new_p = self.b_BOUNDS[0]
                        if (new_p > self.b_BOUNDS[1]):
                            new_p = self.b_BOUNDS[1]
                    elif (i == 11):
                        if (new_p < self.s_BOUNDS[0]):
                            new_p = self.s_BOUNDS[0]
                        if (new_p > self.s_BOUNDS[1]):
                            new_p = self.s_BOUNDS[1]

                    new_pos.append(new_p)
                    i += 1

                # Set the new values
                individual[0:12] = new_pos[0:12]  # 12 element vector
                individual[12:24] = new_vel[0:12]  # 12 element vector
                
                self.population[index] = individual
                
                index += 1
                
            iteration += 1

            # Log the results
            self.log_file.write("\n" + "Results of this iteration: ")
            self.log_file.write('GBest Fitness: ' + str(self.GBest[12]) + ' GBest position: ' + str(self.GBest[0:12]) + '\n' + '\n')


    # Function to generate random position vectors
    def generate_random_individual_positions(self):
        
        # Generate a vector of 12 elements of random positions
        # First 6 elements are network weights
        a1 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        a2 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        a3 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        a4 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        a5 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        a6 = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
        # Tr1 and Tr2
        Tr1 = random.uniform(self.Tr1_BOUNDS[0], self.Tr1_BOUNDS[1])
        Tr2 = random.uniform(self.Tr2_BOUNDS[0], self.Tr2_BOUNDS[1])
        # Ta1 and Ta2
        Ta1 = random.uniform(self.Ta1_BOUNDS[0], self.Ta1_BOUNDS[1])
        Ta2 = random.uniform(self.Ta2_BOUNDS[0], self.Ta2_BOUNDS[1])
        # b and s
        b = random.uniform(self.b_BOUNDS[0], self.b_BOUNDS[1])
        s = random.uniform(self.s_BOUNDS[0], self.s_BOUNDS[1])
        
        return [a1, a2, a3, a4, a5, a6, Tr1, Tr2, Ta1, Ta2, b, s]


    # Function to generate random velocity vectors
    def generate_random_individual_velocities(self):
        
        # Generate a vector of 12 elements of random positions
        # First 6 elements are network weights
        a1 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        a2 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        a3 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        a4 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        a5 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        a6 = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
        # Tr1 and Tr2
        Tr1 = random.uniform(self.VEL_BOUND_PCT * self.Tr1_BOUNDS[0], self.VEL_BOUND_PCT * self.Tr1_BOUNDS[1])
        Tr2 = random.uniform(self.VEL_BOUND_PCT * self.Tr2_BOUNDS[0], self.VEL_BOUND_PCT * self.Tr2_BOUNDS[1])
        # Ta1 and Ta2
        Ta1 = random.uniform(self.VEL_BOUND_PCT * self.Ta1_BOUNDS[0], self.VEL_BOUND_PCT * self.Ta1_BOUNDS[1])
        Ta2 = random.uniform(self.VEL_BOUND_PCT * self.Ta2_BOUNDS[0], self.VEL_BOUND_PCT * self.Ta2_BOUNDS[1])
        # b and s
        b = random.uniform(self.VEL_BOUND_PCT * self.b_BOUNDS[0], self.VEL_BOUND_PCT * self.b_BOUNDS[1])
        s = random.uniform(self.VEL_BOUND_PCT * self.s_BOUNDS[0], self.VEL_BOUND_PCT * self.s_BOUNDS[1])
        
        return [a1, a2, a3, a4, a5, a6, Tr1, Tr2, Ta1, Ta2, b, s]

if __name__ == '__main__':
    DarwinWalkOptimization()



