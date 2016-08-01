#!/usr/bin/env python

###############################################################################
# File: darwin_walk_optimization_symmetric.py
# Description: Evolves neural oscillators for walking on flat terrain using PSO
#              Here the symmetry of the 2 legs is considered
# Execution: rosrun robot_walking darwin_walk_optimization.py
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


class DarwinWalkOptimization():

    def __init__(self):

        ################################## Constant Definitions ##################################

        # Matsuoka Oscillator related initialization

        self.WEIGHT_BOUNDS = [1.0, 3.5]  # Weights of the neural oscillator connections
        self.Tr_BOUNDS = [0.1, 2.0]  # Rise time constant
        self.Ta_BOUNDS = [0.1, 2.0]  # Adaptation time constant
        self.b_BOUNDS = [0.1, 2.0]  # Constant of the Matsuoka Oscillator
        self.s_BOUNDS = [1.0, 2.0]  # Constant of the Matsuoka Oscillator
        self.NUM_WEIGHTS = 7  # Number of distinct weights in the network
        self.VEL_BOUND_PCT = 0.2  # Velocity is at most 20% of the position range
        self.TRIAL_DURATION = 30  # How many seconds should each trial last
        self.STEP_DURATION = 0.2  # The step increment in seconds

        # PSO specific initialization

        self.POPULATION_SIZE = 20
        # Each individual consists of 11 positions (weights, and oscillator constants),
        # 11 velocities, 11 PBest positions and 1 PBest score
        self.INDIVIDUAL_VECTOR_SIZE = 34
        self.GBEST_VECTOR_SIZE = 12
        self.MAX_ITERS = 10
        self.C1 = 2.0  # Acceleration constant
        self.C2 = 2.0  # Acceleration constant
        # Inertial weight, linearly scales from 0.9 to 0.4 over the entire run
        self.INERTIAL_WEIGHT_BOUNDS = [0.4, 0.9]

        # Darwin specific initialization

        # Hashmap of joint origins
        self.joint_origins = {}
        self.joint_origins['j_thigh2_l'] = 0.611
        self.joint_origins['j_tibia_l'] = -1.134
        self.joint_origins['j_ankle1_l'] = 0.0
        self.joint_origins['j_thigh2_r'] = -0.611
        self.joint_origins['j_tibia_r'] = 1.134
        self.joint_origins['j_ankle1_r'] = 0.0

        # Hashmap of joint upper limits
        self.joint_upper_limits = {}
        self.joint_upper_limits['j_thigh2_l'] = 1.745
        self.joint_upper_limits['j_tibia_l'] = 0.0
        self.joint_upper_limits['j_ankle1_l'] = 1.047
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
                             + str(curr_time) + '_darwin_walk_optimization_symmetric.log', 'w')
        self.log_file.write('Created log file at ' + str(curr_time) + '\n')

        # Initiate the walk optimization
        self.pso()

    # Function to generate individuals (network weights and parameters)
    def generate_individual_position(self, Tr_bounds, Ta_bounds, b_bounds, s_bounds, weight_bounds):

        Tr = random.uniform(Tr_bounds[0], Tr_bounds[1])
        Ta = random.uniform(Ta_bounds[0], Ta_bounds[1])
        b = random.uniform(b_bounds[0], b_bounds[1])
        s = random.uniform(s_bounds[0], s_bounds[1])

        # Setting the couplings of the 2 hip joints
        # Generating weights used in network of 4 oscillators
        # This is the weight in each connection of the 4 neuron network
        # The lower limit is the greatest of the numbers the weight should be greater than
        # The upper limit is the smallest of the numbers the weight should be smaller than
        a = random.uniform(max(b, (1 + (Tr / Ta)), weight_bounds[0]), min(((1 + b) / 2.0), weight_bounds[1]))

        # All weights in for the hip neurons are symmetric
        a1_2 = a1_3 = a3_1 = a3_4 = a4_3 = a4_2 = a2_4 = a2_1 = a

        # Generating the coupling of the knee joints
        # The upper limit should be satisfied and the product of the weights should be above a limit
        a5_6 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
        a6_5 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))

        '''while ((a5_6 * a6_5) < (1 + (Tr/Ta))**2):
            print "stuck"
            a5_6 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
            a6_5 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
        '''

        # The knee neuron couplings in the other leg are symmetric
        a7_8 = a5_6
        a8_7 = a6_5

        # Generating the coupling of the ankle joints
        # The upper limit should be satisfied and the product of the weights should be above a limit
        a9_10 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
        a10_9 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))

        '''while ((a9_10 * a10_9) < (1 + (Tr/Ta))**2):
            a9_10 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
            a10_9 = random.uniform(weight_bounds[0], min((1 + b), weight_bounds[1]))
        '''

        # The knee neuron couplings in the other leg are symmetric
        a11_12 = a9_10
        a12_11 = a10_9

        # Generating the coupling weight between hip and knee
        a1_5 = random.uniform(weight_bounds[0], weight_bounds[1])
        a2_7 = a1_5

        # Generating the coupling weight between hip and ankle
        a1_10 = random.uniform(weight_bounds[0], weight_bounds[1])
        a2_12 = a1_10

        return [a, a5_6, a6_5, a9_10, a10_9, a1_5, a1_10, Tr, Ta, b, s]


    # Function to calculate the next state of the Matsuoka neural oscillators
    def matsuoka(self, individual, state):

        print "State: " + str(state)
        print "Individual: " + str(individual)

        # Extract the network weights and parameters
        a1_3 = a3_1 = a3_4 = a4_3 = a4_2 = a2_4 = a2_1 = a1_2 = individual[0]
        a5_6 = a7_8 = individual[1]
        a6_5 = a8_7 = individual[2]
        a9_10 = a11_12 = individual[3]
        a10_9 = a12_11 = individual[4]
        a1_5 = a2_7 = individual[5]
        a1_10 = a2_12 = individual[6]
        Tr = individual[7]
        Ta = individual[8]
        b = individual[9]
        s = individual[10]

        # Extract the current states
        # The numbering scheme: For f21 <variable=f><oscillator#=2><1 for extensor, 2 for flexor>
        # Each oscillator has the state [x1,x2,f1,f2,y1,y2]
        # There are 6 such oscillators
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

        # Calculate the derivatives of the membrane potentials
        # This is based on the connections between the individual neurons

        # Hip neurons
        x1_d = ((-1.0 * a3_1 * y3) - (a2_1 * y2) + (s) - (b * f1) - (x1)) / float(Tr)
        x3_d = ((-1.0 * a1_3 * y1) - (a4_3 * y4) + (s) - (b * f3) - (x3)) / float(Tr)
        x4_d = ((-1.0 * a3_4 * y3) - (a2_4 * y2) + (s) - (b * f4) - (x4)) / float(Tr)
        x2_d = ((-1.0 * a4_2 * y4) - (a1_2 * y1) + (s) - (b * f2) - (x2)) / float(Tr)

        # Knee neurons
        x5_d = ((-1.0 * a6_5 * y6) - (a1_5 * y1) + (s) - (b * f5) - (x5)) / float(Tr)
        x6_d = ((-1.0 * a5_6 * y5) + (s) - (b * f6) - (x6)) / float(Tr)
        x7_d = ((-1.0 * a8_7 * y8) - (a2_7 * y2) + (s) - (b * f7) - (x7)) / float(Tr)
        x8_d = ((-1.0 * a7_8 * y7) + (s) - (b * f8) - (x8)) / float(Tr)

        # Ankle neurons
        x9_d = ((-1.0 * a10_9 * y10) + (s) - (b * f9) - (x9)) / float(Tr)
        x10_d = ((-1.0 * a9_10 * y9) - (a1_10 * y1) + (s) - (b * f10) - (x10)) / float(Tr)
        x11_d = ((-1.0 * a12_11 * y12) + (s) - (b * f11) - (x11)) / float(Tr)
        x12_d = ((-1.0 * a11_12 * y11) - (a2_12 * y2) + (s) - (b * f12) - (x12)) / float(Tr)

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

        joint_plot_1 = []
        t1 = []
        t2 = []
        joint_plot_2 = []

        print 'Received Individual:' + str(individual)

        # Reset the simulation before each walk
        self.reset_simulation()

        # Set initial state - [x1-12, f1-12, y1-12]
        oscillator_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # x1-12
                            0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,  # f1-12
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # y1-12

        # Set the shoulder roll angle and elbow so that arms are at the side
        new_angles = {}
        new_angles['j_high_arm_l'] = 1.50
        new_angles['j_high_arm_r'] = 1.50
        new_angles['j_low_arm_l'] = 0.0
        new_angles['j_low_arm_r'] = 0.0
        self.darwin.set_angles_slow(new_angles)

        # Calculate the number of iterations
        num_iterations = self.TRIAL_DURATION / self.STEP_DURATION

        i = 0
        while (i < num_iterations):

            # Compute the next state from the Matsuoka oscillator
            # Pass the individual (with weights and constants)
            oscillator_state = self.matsuoka(individual, oscillator_state)

            # Unpack the state vector
            y1 = oscillator_state[24]
            y2 = oscillator_state[25]
            y3 = oscillator_state[26]
            y4 = oscillator_state[27]
            y5 = oscillator_state[28]
            y6 = oscillator_state[29]
            y7 = oscillator_state[30]
            y8 = oscillator_state[31]
            y9 = oscillator_state[32]
            y10 = oscillator_state[33]
            y11 = oscillator_state[34]
            y12 = oscillator_state[35]

            # Right Leg
            # neuron 1+3   => j_thigh2_r (hip)
            # neuron 5+6   => j_tibia_r  (knee)
            # neuron 9+10  => j_ankle1_r (ankle)

            # Left Leg
            # neuron 2+4   => j_thigh2_l (hip)
            # neuron 7+8   => j_tibia_l  (knee)
            # neuron 11+12 => j_ankle1_l (ankle

            # Shift the positions to the joint origins
            joint_j_thigh2_r = y1 - y3
            joint_j_tibia_r  = y5 - y6
            joint_j_ankle1_r = y9 - y10

            joint_j_thigh2_l = y1 - y3
            joint_j_tibia_l  = (y7 - y8)
            joint_j_ankle1_l = (y11 - y12)


            # If joint angles exceed the joint limits, set the joint limit as the joint angle
            joint_j_thigh2_l = self.joint_upper_limits['j_thigh2_l'] if (joint_j_thigh2_l > self.joint_upper_limits['j_thigh2_l']) else joint_j_thigh2_l
            joint_j_thigh2_l = self.joint_lower_limits['j_thigh2_l'] if (joint_j_thigh2_l < self.joint_lower_limits['j_thigh2_l']) else joint_j_thigh2_l
            joint_j_tibia_l = self.joint_upper_limits['j_tibia_l']   if (joint_j_tibia_l > self.joint_upper_limits['j_tibia_l'])   else joint_j_tibia_l
            joint_j_tibia_l = self.joint_lower_limits['j_tibia_l']   if (joint_j_tibia_l < self.joint_lower_limits['j_tibia_l'])   else joint_j_tibia_l
            joint_j_ankle1_l = self.joint_upper_limits['j_ankle1_l'] if (joint_j_ankle1_l > self.joint_upper_limits['j_ankle1_l']) else joint_j_ankle1_l
            joint_j_ankle1_l = self.joint_lower_limits['j_ankle1_l'] if (joint_j_ankle1_l < self.joint_lower_limits['j_ankle1_l']) else joint_j_ankle1_l
            joint_j_thigh2_r = self.joint_upper_limits['j_thigh2_r'] if (joint_j_thigh2_r > self.joint_upper_limits['j_thigh2_r']) else joint_j_thigh2_r
            joint_j_thigh2_r = self.joint_lower_limits['j_thigh2_r'] if (joint_j_thigh2_r < self.joint_lower_limits['j_thigh2_r']) else joint_j_thigh2_r
            joint_j_tibia_r = self.joint_upper_limits['j_tibia_r']   if (joint_j_tibia_r > self.joint_upper_limits['j_tibia_r'])   else joint_j_tibia_r
            joint_j_tibia_r = self.joint_lower_limits['j_tibia_r']   if (joint_j_tibia_r < self.joint_lower_limits['j_tibia_r'])   else joint_j_tibia_r
            joint_j_ankle1_r = self.joint_upper_limits['j_ankle1_r'] if (joint_j_ankle1_r > self.joint_upper_limits['j_ankle1_r']) else joint_j_ankle1_r
            joint_j_ankle1_r = self.joint_lower_limits['j_ankle1_r'] if (joint_j_ankle1_r < self.joint_lower_limits['j_ankle1_r']) else joint_j_ankle1_r


            print "Joint angles:"
            print "[[" + str(joint_j_thigh2_l) + ", " + str(joint_j_tibia_l) + ", " + str(
                joint_j_ankle1_l) + ", " + str(joint_j_thigh2_r) + ", " + str(joint_j_tibia_r) + ", " + str(
                joint_j_ankle1_r) + "]]"

            if (i * self.STEP_DURATION > 5.0):
                # Set the new angles
                new_angles['j_thigh2_l'] = joint_j_thigh2_l
                new_angles['j_tibia_l'] = joint_j_tibia_l
                new_angles['j_ankle1_l'] = joint_j_ankle1_l


                new_angles['j_thigh2_r'] = joint_j_thigh2_r
                new_angles['j_tibia_r'] = joint_j_tibia_r
                new_angles['j_ankle1_r'] = joint_j_ankle1_r


                joint_plot_1.append(joint_j_thigh2_r)
                joint_plot_2.append(joint_j_thigh2_r)
                t1.append(i*self.STEP_DURATION)

                # The last parameter is the delay, this should match the step size
                self.darwin.set_angles_slow(new_angles, self.STEP_DURATION)

                # Calculate the current distance travelled by the robot #np.sign(self.current_model_x) *
                self.distance_walked = math.sqrt((self.current_model_x)**2 + (self.current_model_y)**2)

                # If the robot has fallen, return the distance travelled till now as the fitness score
                if (self.has_fallen):
                    #plt.figure()
                    #plt.plot(t1, joint_plot_1)
                    #plt.plot(t1, joint_plot_2)
                    #plt.show()
                    return self.distance_walked

            i = i + 1

        # If the trial duration has ended return the distance travelled as the fitness score
        return self.distance_walked

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
        new_angles['j_tilt'] = 0.0
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
            print "Darwin model state initialization service call failed: %s" % e

        rospy.loginfo("Result of model reset: " + str(response))

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
            print "Service call failed: %s" % e

    # Function for Particle Swarm Optimization
    def pso(self):

        # PSO specific variables

        # The GBest vector - The first 11 elements are positions, the last element is
        # the fitness score
        self.GBest = []
        index = 0
        while (index < self.GBEST_VECTOR_SIZE):
            self.GBest.append(0.0)
            index += 1

        # List of individual vectors
        self.population = []

        # Initialize the population
        index = 0
        while (index < self.POPULATION_SIZE):

            # First 11 elements are randomized positions
            # 7 weights + 4 constants of the neural oscillators
            individual = self.generate_individual_position(self.Tr_BOUNDS,
                                                  self.Ta_BOUNDS,
                                                  self.b_BOUNDS,
                                                  self.s_BOUNDS,
                                                  self.WEIGHT_BOUNDS)

            # Next 11 elements are randomized bounded velocities
            i = 0
            # Create random velocities
            while (i < self.NUM_WEIGHTS):
                # Randomly create a bounded weight and add to the individual vector
                # Velocity range is VEL_BOUND_PCT% of the dynamic position range
                v = random.uniform(self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0],
                                   self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1])
                individual.append(v)
                i += 1
            v_Tr = random.uniform(self.VEL_BOUND_PCT * self.Tr_BOUNDS[0], self.VEL_BOUND_PCT * self.Tr_BOUNDS[1])
            v_Ta = random.uniform(self.VEL_BOUND_PCT * self.Ta_BOUNDS[0], self.VEL_BOUND_PCT * self.Ta_BOUNDS[1])
            v_b = random.uniform(self.VEL_BOUND_PCT * self.b_BOUNDS[0], self.VEL_BOUND_PCT * self.b_BOUNDS[1])
            v_s = random.uniform(self.VEL_BOUND_PCT * self.s_BOUNDS[0], self.VEL_BOUND_PCT * self.s_BOUNDS[1])
            individual.append(v_Tr)
            individual.append(v_Ta)
            individual.append(v_b)
            individual.append(v_s)

            # Next 11 elements are pbest positions, set to the initial positions
            i = 0
            while (i < 11):
                individual.append(individual[i])
                i += 1

            # The last element is the pbest fitness, 0.0 initially
            individual.append(0.0)

            # Add the individual to the population
            self.population.append(individual)

            # On to the next individual
            index += 1

        # Main PSO logic
        iteration = 0
        while (iteration < self.MAX_ITERS):

            # Set the linearly scaled inertial weight
            w = self.INERTIAL_WEIGHT_BOUNDS[1] - (iteration * (self.INERTIAL_WEIGHT_BOUNDS[1] - self.INERTIAL_WEIGHT_BOUNDS[0]) / self.MAX_ITERS)

            index = 0
            for individual in self.population:

                # Calculate fitness of individual
                indiv_fitness = self.matsuoka_walking_fitness(individual)

                # Check pbest (last element of the individual vector)
                if (indiv_fitness > individual[33]):
                    # Set the individual's pbest as the current fitness
                    individual[33] = indiv_fitness
                    # Copy the current position into the individual's pbest position
                    individual[22:32] = individual[0:11]
                    # Set the individual in the population
                    self.population[index] = individual

                # Check gbest and set if the current fitness is better
                if (indiv_fitness > self.GBest[11]):
                    self.GBest[11] = indiv_fitness
                    self.GBest[0:10] = individual[0:10]

                index += 1

            # Once pbest and gbest have been determined, position and velocities need to be updated
            index = 0
            for individual in self.population:

                # Unpack the vector
                pos = individual[0:10]  # 11 element vector
                vel = individual[11:21]  # 11 element vector
                pbest = individual[22:32]  # 11 element vector
                new_pos = []
                new_vel = []

                # Generate 2 random numbers
                rand1 = random.random()
                rand2 = random.random()

                # Calculate new velocities
                i = 0
                for v in vel:
                    new_v = w * v + self.C1 * rand1 * (pbest[i] - pos[i]) + self.C2 * rand2 * (self.GBest[i] - pos[i])

                    # Bound checking
                    # For the network weights
                    if (i >= 0 and i < 7):
                        if (new_v < self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif (i == 7):
                        if (new_v < self.VEL_BOUND_PCT * self.Tr_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Tr_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Tr_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Tr_BOUNDS[1]
                    elif (i == 8):
                        if (new_v < self.VEL_BOUND_PCT * self.Ta_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.Ta_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.Ta_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.Ta_BOUNDS[1]
                    elif (i == 9):
                        if (new_v < self.VEL_BOUND_PCT * self.b_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT * self.b_BOUNDS[0]
                        if (new_v > self.VEL_BOUND_PCT * self.b_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT * self.b_BOUNDS[1]
                    elif (i == 10):
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

                    # Bound checking
                    # For the network weights
                    if (i >= 0 and i < 7):
                        if (new_p < self.WEIGHT_BOUNDS[0]):
                            new_p = self.WEIGHT_BOUNDS[0]
                        if (new_p > self.WEIGHT_BOUNDS[1]):
                            new_p = self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif (i == 7):
                        if (new_p < self.Tr_BOUNDS[0]):
                            new_p = self.Tr_BOUNDS[0]
                        if (new_p > self.Tr_BOUNDS[1]):
                            new_p = self.Tr_BOUNDS[1]
                    elif (i == 8):
                        if (new_p < self.Ta_BOUNDS[0]):
                            new_p = self.Ta_BOUNDS[0]
                        if (new_p > self.Ta_BOUNDS[1]):
                            new_p = self.Ta_BOUNDS[1]
                    elif (i == 9):
                        if (new_p < self.b_BOUNDS[0]):
                            new_p = self.b_BOUNDS[0]
                        if (new_p > self.b_BOUNDS[1]):
                            new_p = self.b_BOUNDS[1]
                    elif (i == 10):
                        if (new_p < self.s_BOUNDS[0]):
                            new_p = self.s_BOUNDS[0]
                        if (new_p > self.s_BOUNDS[1]):
                            new_p = self.s_BOUNDS[1]

                    new_pos.append(new_p)
                    i += 1

                # Set the new values
                individual[0:10] = new_pos[0:10]  # 11 element vector
                individual[11:21] = new_vel[0:10]  # 11 element vector

                self.population[index] = individual

                index += 1

            iteration += 1

            # Log the results
            curr_time = time.strftime('%Y%m%d%H%M%S')
            self.log_file.write(
                '[' + str(curr_time) + ']' + ' GBest Fitness: ' + str(self.GBest[11]) + ' GBest position: ' + str(
                    self.GBest[0:11]) + '\n')


if __name__ == '__main__':
    DarwinWalkOptimization()
