#!/usr/bin/env python

###############################################################################
# File: darwin_walk_optimization.py
# Description: Evolves neural oscillators for walking on flat terrain using PSO
# Execution: rosrun robot_walking darwin_walk_optimization.py
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time


class Darwin_Walk_Optimization():

    
    def __init__(self):


        ################################## Constant Definitions ################################## 
        
        # Matsuoka Oscillator related initialization

        self.WEIGHT_BOUNDS = [-512.0,512.0] # Weights of the neural oscillator connections
        self.Tr_BOUNDS = [0.01,0.075]       # Rise time constant
        self.Ta_BOUNDS = [0.04,0.75]        # Adaptation time constant
        self.b_BOUNDS = [1.25,2.0]          # Constant of the Matsuoka Oscillator
        self.s_BOUNDS = [0.0,5.0]           # Constant of the Matsuoka Oscillator
        self.NUM_WEIGHTS = 20               # Number of weights in the network
        self.VEL_BOUND_PCT = 0.2            # Velocity is at most 20% of the position range
        self.TRIAL_DURATION = 30            # How many seconds should each trial last
        self.STEP_DURATION = 0.2            # The step increment in seconds
        
        
        # PSO specific initialization
        
        self.POPULATION_SIZE = 100
        # Each individual consists of 24 positions (weights, and oscillator constants), 
        # 24 velocities, 24 PBest positions and 1 PBest score
        self.INDIVIDUAL_VECTOR_SIZE = 73
        self.GBEST_VECTOR_SIZE = 25
        self.MAX_ITERS = 200
        self.C1 = 2.0 # Acceleration constant
        self.C2 = 2.0 # Acceleration constant
        # Inertial weight, linearly scales from 0.9 to 0.4 over the entire run
        self.INERTIAL_WEIGHT_BOUNDS = [0.4,0.9] 

        
        # Darwin specific initialization
        
        # Hashmap of joint origins
        self.joint_origins = {}
        self.joint_origins['j_thigh2_l'] = 0.611 
        self.joint_origins['j_tibia_l']  = -1.134
        self.joint_origins['j_ankle1_l'] = 0.0
        self.joint_origins['j_thigh2_r'] = -0.611
        self.joint_origins['j_tibia_r']  = 1.134
        self.joint_origins['j_ankle1_r'] = 0.0
        
        # Hashmap of joint upper limits
        self.joint_upper_limits = {}
        self.joint_upper_limits['j_thigh2_l'] = 1.745
        self.joint_upper_limits['j_tibia_l']  = 0.0
        self.joint_upper_limits['j_ankle1_l'] = 1.047
        self.joint_upper_limits['j_thigh2_r'] = 0.524
        self.joint_upper_limits['j_tibia_r']  = 2.269
        self.joint_upper_limits['j_ankle1_r'] = 1.047

        # Hashmap of joint lower limits
        self.joint_lower_limits = {}
        self.joint_lower_limits['j_thigh2_l'] = -0.524
        self.joint_lower_limits['j_tibia_l']  = -2.269
        self.joint_lower_limits['j_ankle1_l'] = -1.047
        self.joint_lower_limits['j_thigh2_r'] = -1.745
        self.joint_lower_limits['j_tibia_r']  = 0.0
        self.joint_lower_limits['j_ankle1_r'] = -1.047        

        # To implement:
        # Mechanism to stop trial if robot falls
        # Reset joint positions
        # Return distance travelled as a fitness value
        
        ########################################################################################## 
        
        self.pso()

    # Function to calculate the next state of the Matsuoka neural oscillators
    def matsuoka(self, individual, state):

        # Extract the weights
        w0  = individual[0]        
        w1  = individual[1]
        w2  = individual[2]
        w3  = individual[3]
        w4  = individual[4]
        w5  = individual[5]
        w6  = individual[6]
        w7  = individual[7]
        w8  = individual[8]
        w9  = individual[9]
        w10 = individual[10]
        w11 = individual[11]
        w12 = individual[12]
        w13 = individual[13]
        w14 = individual[14]
        w15 = individual[15]
        w16 = individual[16]
        w17 = individual[17]
        w18 = individual[18]   
        w19 = individual[19]      
        
        # Extract the network constants
        Tr = individual[20]
        Ta = individual[21]
        b  = individual[22]
        s  = individual[23]                                                                                                             
       
        # Extract the current states
        # The numbering scheme: For f21 <variable=f><oscillator#=2><1 for extensor, 2 for flexor>
        # Each oscillator has the state [x1,x2,f1,f2,y1,y2]
        # There are 6 such oscillators
        x11 = state[0][0] 
        x12 = state[0][1] 
        f11 = state[0][2] 
        f12 = state[0][3] 
        y11 = state[0][4]
        y12 = state[0][5]

        x21 = state[1][0] 
        x22 = state[1][1] 
        f21 = state[1][2] 
        f22 = state[1][3] 
        y21 = state[1][4]
        y22 = state[1][5]
    
        x31 = state[2][0] 
        x32 = state[2][1] 
        f31 = state[2][2] 
        f32 = state[2][3] 
        y31 = state[2][4]
        y32 = state[2][5]

        x41 = state[3][0] 
        x42 = state[3][1] 
        f41 = state[3][2] 
        f42 = state[3][3] 
        y41 = state[3][4]
        y42 = state[3][5]

        x51 = state[4][0] 
        x52 = state[4][1] 
        f51 = state[4][2] 
        f52 = state[4][3] 
        y51 = state[4][4]
        y52 = state[4][5]

        x61 = state[5][0] 
        x62 = state[5][1] 
        f61 = state[5][2] 
        f62 = state[5][3] 
        y61 = state[5][4]
        y62 = state[5][5]
        
        
        # Calculate the derivatives of the membrane potentials
        # This is based on the connections between the individual neurons
        x11_d = ((-1.0*w1*y12) - (w6*y21) + (s) - (b*f11) - (x11))/float(Tr)
        x12_d = ((-1.0*w2*y11) - (w8*y22) + (s) - (b*f12) - (x12))/float(Tr)
        
        x21_d = ((-1.0*w3*y22) - (w5*y11) + (s) - (b*f21) - (x21))/float(Tr)
        x22_d = ((-1.0*w4*y21) - (w7*y12) + (s) - (b*f22) - (x22))/float(Tr)
        
        x31_d = ((-1.0*w13*y32) - (w9*y11) + (s) - (b*f31) - (x31))/float(Tr)
        x32_d = ((-1.0*w14*y31) + (s) - (b*f32) - (x32))/float(Tr)

        x41_d = ((-1.0*w15*y42) - (w10*y21) + (s) - (b*f41) - (x41))/float(Tr)
        x42_d = ((-1.0*w16*y41) + (s) - (b*f42) - (x42))/float(Tr)

        x51_d = ((-1.0*w17*y52) + (s) - (b*f51) - (x51))/float(Tr)
        x52_d = ((-1.0*w18*y51) - (w11*y11) + (s) - (b*f52) - (x52))/float(Tr)

        x61_d = ((-1.0*w19*y62) + (s) - (b*f61) - (x61))/float(Tr)
        x62_d = ((-1.0*w20*y61) - (w12*y21) + (s) - (b*f62) - (x62))/float(Tr)

        # Calculate the elements of the next state        
        x11 = x11 + self.STEP_DURATION*x11_d
        x12 = x12 + self.STEP_DURATION*x12_d
        x21 = x21 + self.STEP_DURATION*x21_d
        x22 = x22 + self.STEP_DURATION*x22_d
        x31 = x31 + self.STEP_DURATION*x31_d
        x32 = x32 + self.STEP_DURATION*x32_d
        x41 = x41 + self.STEP_DURATION*x41_d
        x42 = x42 + self.STEP_DURATION*x42_d
        x51 = x51 + self.STEP_DURATION*x51_d
        x52 = x52 + self.STEP_DURATION*x52_d
        x61 = x61 + self.STEP_DURATION*x61_d
        x62 = x62 + self.STEP_DURATION*x62_d
                        
        y11 = max(0.0, x11)
        y12 = max(0.0, x12)
        y21 = max(0.0, x21)
        y22 = max(0.0, x22)
        y31 = max(0.0, x31)
        y32 = max(0.0, x32)
        y41 = max(0.0, x41)
        y42 = max(0.0, x42)
        y51 = max(0.0, x51)
        y52 = max(0.0, x52)
        y61 = max(0.0, x61)
        y62 = max(0.0, x62)
                
        f11_d = (y11 - f11)/float(Ta)
        f12_d = (y12 - f12)/float(Ta)
        f21_d = (y21 - f21)/float(Ta)
        f22_d = (y22 - f22)/float(Ta)
        f31_d = (y31 - f31)/float(Ta)
        f32_d = (y32 - f32)/float(Ta)
        f41_d = (y41 - f41)/float(Ta)
        f42_d = (y42 - f42)/float(Ta)
        f51_d = (y51 - f51)/float(Ta)
        f52_d = (y52 - f52)/float(Ta)
        f61_d = (y61 - f61)/float(Ta)
        f62_d = (y62 - f62)/float(Ta)
        
        f11 = f11 + self.STEP_DURATION*f11_d
        f12 = f12 + self.STEP_DURATION*f12_d       
        f21 = f21 + self.STEP_DURATION*f21_d
        f22 = f22 + self.STEP_DURATION*f22_d       
        f31 = f31 + self.STEP_DURATION*f31_d
        f32 = f32 + self.STEP_DURATION*f32_d       
        f41 = f41 + self.STEP_DURATION*f41_d
        f42 = f42 + self.STEP_DURATION*f42_d       
        f51 = f51 + self.STEP_DURATION*f51_d
        f52 = f52 + self.STEP_DURATION*f52_d       
        f61 = f61 + self.STEP_DURATION*f61_d
        f62 = f62 + self.STEP_DURATION*f62_d       
        
        # Create the return state
        ret_state = [x11,x12,f11,f12,y11,y12,
                     x21,x22,f21,f22,y21,y22,
                     x31,x32,f31,f32,y31,y32,
                     x41,x42,f41,f42,y41,y42,
                     x51,x52,f51,f52,y51,y52,
                     x61,x62,f61,f62,y61,y62
                    ] 
        
        return ret_state
        

    # Function for Walking using Matsuoka Oscillator
    def matsuoka_walking_fitness(self,individual,TRIAL_DURATION):
    
        # Set initial state - [x1,x2,f1,f2,y1,y2] for each oscillator
        # There are 6 oscillators, one each for the joints j_thigh2_l,j_tibia_l,j_ankle1_l,j_thigh2_r,j_tibia_r,j_ankle1_r

        oscillator_state = []
        count = 0
        while(count<6):
            oscillator_state.extend([0.0, 1.0, 0.0, 1.0, 0.0 ,0.0])
            count += 1
            
        
        # Set the shoulder roll angle and elbow so that arms are at the side
        new_angles = {}
        new_angles['j_high_arm_l'] = 1.50
        new_angles['j_high_arm_r'] = 1.50
        new_angles['j_low_arm_l'] = 0.0
        new_angles['j_low_arm_r'] = 0.0
        self.darwin.set_angles_slow(new_angles)
        
        # Calculate the number of iterations
        num_iterations = self.TRIAL_DURATION/self.STEP_DURATION

        i=0
        while(i<num_iterations):
        
            # Compute the next state from the Matsuoka oscillator
            # Pass the individual (with weights and constants)
            oscillator_state = self.matsuoka(individual,oscillator_state)
            
            # Shift the positions to the joint origins
            oscillator_joint_1 = (oscillator_state[0*6 + 4] - oscillator_state[0*6 + 5]) + self.joint_origins['j_thigh2_l']
            oscillator_joint_2 = (oscillator_state[1*6 + 4] - oscillator_state[1*6 + 5]) + self.joint_origins['j_tibia_l']
            oscillator_joint_3 = (oscillator_state[2*6 + 4] - oscillator_state[2*6 + 5]) + self.joint_origins['j_ankle1_l']
            oscillator_joint_4 = (oscillator_state[3*6 + 4] - oscillator_state[3*6 + 5]) + self.joint_origins['j_thigh2_r']
            oscillator_joint_5 = (oscillator_state[4*6 + 4] - oscillator_state[4*6 + 5]) + self.joint_origins['j_tibia_r']
            oscillator_joint_6 = (oscillator_state[5*6 + 4] - oscillator_state[5*6 + 5]) + self.joint_origins['j_ankle1_r']
                        
            # If joint angles exceed the joint limits, set the joint limit as the joint angle            
            oscillator_joint_1 = self.joint_upper_limits['j_thigh2_l'] if (oscillator_joint_1>self.joint_upper_limits['j_thigh2_l']) else oscillator_joint_1
            oscillator_joint_1 = self.joint_lower_limits['j_thigh2_l'] if (oscillator_joint_1<self.joint_lower_limits['j_thigh2_l']) else oscillator_joint_1                        
            oscillator_joint_2 = self.joint_upper_limits['j_tibia_l']  if (oscillator_joint_2>self.joint_upper_limits['j_tibia_l'])  else oscillator_joint_2
            oscillator_joint_2 = self.joint_lower_limits['j_tibia_l']  if (oscillator_joint_2<self.joint_lower_limits['j_tibia_l'])  else oscillator_joint_2            
            oscillator_joint_3 = self.joint_upper_limits['j_ankle1_l'] if (oscillator_joint_3>self.joint_upper_limits['j_ankle1_l']) else oscillator_joint_3
            oscillator_joint_3 = self.joint_lower_limits['j_ankle1_l'] if (oscillator_joint_3<self.joint_lower_limits['j_ankle1_l']) else oscillator_joint_3            
            oscillator_joint_4 = self.joint_upper_limits['j_thigh2_r'] if (oscillator_joint_4>self.joint_upper_limits['j_thigh2_r']) else oscillator_joint_4
            oscillator_joint_4 = self.joint_lower_limits['j_thigh2_r'] if (oscillator_joint_4<self.joint_lower_limits['j_thigh2_r']) else oscillator_joint_4            
            oscillator_joint_5 = self.joint_upper_limits['j_tibia_r']  if (oscillator_joint_5>self.joint_upper_limits['j_tibia_r'])  else oscillator_joint_5
            oscillator_joint_5 = self.joint_lower_limits['j_tibia_r']  if (oscillator_joint_5<self.joint_lower_limits['j_tibia_r'])  else oscillator_joint_5            
            oscillator_joint_6 = self.joint_upper_limits['j_ankle1_r'] if (oscillator_joint_6>self.joint_upper_limits['j_ankle1_r']) else oscillator_joint_6
            oscillator_joint_6 = self.joint_lower_limits['j_ankle1_r'] if (oscillator_joint_6<self.joint_lower_limits['j_ankle1_r']) else oscillator_joint_6            
           
            # Set the new angles
            new_angles['j_thigh2_l'] = oscillator_joint_1
            new_angles['j_tibia_l']  = oscillator_joint_2
            new_angles['j_ankle1_l'] = oscillator_joint_3
            new_angles['j_thigh2_r'] = oscillator_joint_4
            new_angles['j_tibia_r']  = oscillator_joint_5
            new_angles['j_ankle1_r'] = oscillator_joint_6                                                            
                        
            # The last parameter is the delay, this should match the step size
            self.darwin.set_angles_slow(new_angles,self.STEP_DURATION)
            
            i=i+1
        

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



    # Function for Particle Swarm Optimization
    def pso(self):
        

        # PSO specific variables
        
        # The GBest vector - The first 24 elements are positions, the last element is 
        # the fitness score
        self.GBest = []    
        index = 0
        while(index<self.GBEST_VECTOR_SIZE):
            self.GBest.append(0.0)
            index += 1

        # List of individual vectors
        self.population = []

        # Initialize the population    
        index = 0
        while(index<self.POPULATION_SIZE):

            # First 24 elements are randomized positions
            # 20 weights + 4 constants of the neural oscillators
            individual = []
            i = 0
            # Create random positions
            while(i<self.NUM_WEIGHTS):
                # Randomly create a bounded weight and add to the individual vector
                w = random.uniform(self.WEIGHT_BOUNDS[0], self.WEIGHT_BOUNDS[1])
                individual.append(w)
                i += 1      
            Tr = random.uniform(self.Tr_BOUNDS[0], self.Tr_BOUNDS[1])
            Ta = random.uniform(self.Ta_BOUNDS[0], self.Ta_BOUNDS[1])
            b  = random.uniform(self.b_BOUNDS[0], self.b_BOUNDS[1])
            s  = random.uniform(self.s_BOUNDS[0], self.s_BOUNDS[1])
            individual.append(Tr)
            individual.append(Ta)
            individual.append(b)
            individual.append(s)                        
            
            # Next 24 elements are randomized bounded velocities
            i = 0
            # Create random velocities
            while(i<self.NUM_WEIGHTS):
                # Randomly create a bounded weight and add to the individual vector
                # Velocity range is VEL_BOUND_PCT% of the dynamic position range
                v = random.uniform(self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[0], self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[1])
                individual.append(v)
                i += 1
            v_Tr = random.uniform(self.VEL_BOUND_PCT*self.Tr_BOUNDS[0], self.VEL_BOUND_PCT*self.Tr_BOUNDS[1])
            v_Ta = random.uniform(self.VEL_BOUND_PCT*self.Ta_BOUNDS[0], self.VEL_BOUND_PCT*self.Ta_BOUNDS[1])
            v_b  = random.uniform(self.VEL_BOUND_PCT*self.b_BOUNDS[0], self.VEL_BOUND_PCT*self.b_BOUNDS[1])
            v_s  = random.uniform(self.VEL_BOUND_PCT*self.s_BOUNDS[0], self.VEL_BOUND_PCT*self.s_BOUNDS[1])        
            individual.append(v_Tr)
            individual.append(v_Ta)
            individual.append(v_b)
            individual.append(v_s)     
            
            # Next 24 elements are pbest positions, set to the initial positions
            i=0
            while(i<24):
                individual.append(individual[i])
                i += 1
                
            # The last element is the pbest fitness, 0.0 initially           
            individual.append(0.0)
            
            # Add the individual to the population
            self.population.append(individual)
            
            # On to the next individual
            index += 1


                    
        iteration = 0
        while(iteration < self.MAX_ITERS):
            
            # Set the linearly scaled inertial weight
            w = self.INERTIAL_WEIGHT_BOUNDS[1] - (iteration*(self.INERTIAL_WEIGHT_BOUNDS[1] - self.INERTIAL_WEIGHT_BOUNDS[0])/self.MAX_ITERS)

            index = 0
            for individual in self.population:
                            
                # Calculate fitness of individual
                indiv_fitness = self.matsuoka_walking_fitness(individual, self.TRIAL_DURATION)

                # Check pbest (last element of the individual vector)
                if(indiv_fitness>individual[72]):
                    
                    # Set the individual's pbest as the current fitness
                    individual[72] = indiv_fitness
                    # Copy the current position into the individual's pbest position
                    individual[48:72] = individual[0:24]
                    # Set the individual in the population 
                    self.population[index] = individual
                    
                # Check gbest and set if the current fitness is better
                if(indiv_fitness>self.GBest[24]):
                    self.GBest[24] = indiv_fitness
                    self.GBest[0:24] = individual[0:24]
                
                index += 1
                
            # Once pbest and gbest have been determined, position and velocities need to be updated
            index = 0
            for individual in self.population: 

                # Unpack the vector
                pos = individual[0:24]    # 24 element vector
                vel = individual[24:48]   # 24 element vector
                pbest = individual[48:72] # 24 element vector
                new_pos = []
                new_vel = []
         
            
                # Generate 2 random numbers
                rand1 = random.random()
                rand2 = random.random()
                
                # Calculate new velocities
                i = 0
                for v in vel:
                    new_v = w*v + self.C1*rand1*(pbest[i]-pos[i]) + self.C2*rand2*(self.GBest[i]-pos[i])
                    
                    # Bound checking
                    # For the network weights
                    if(i>=0 and i<20):
                        if(new_v<self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[0]
                        if(new_v>self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT*self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif(i==20):
                        if(new_v<self.VEL_BOUND_PCT*self.Tr_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT*self.Tr_BOUNDS[0]
                        if(new_v>self.VEL_BOUND_PCT*self.Tr_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT*self.Tr_BOUNDS[1]                    
                    elif(i==21):
                        if(new_v<self.VEL_BOUND_PCT*self.Ta_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT*self.Ta_BOUNDS[0]
                        if(new_v>self.VEL_BOUND_PCT*self.Ta_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT*self.Ta_BOUNDS[1]                    
                    elif(i==22):
                        if(new_v<self.VEL_BOUND_PCT*self.b_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT*self.b_BOUNDS[0]
                        if(new_v>self.VEL_BOUND_PCT*self.b_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT*self.b_BOUNDS[1]                    
                    elif(i==23):
                        if(new_v<self.VEL_BOUND_PCT*self.s_BOUNDS[0]):
                            new_v = self.VEL_BOUND_PCT*self.s_BOUNDS[0]
                        if(new_v>self.VEL_BOUND_PCT*self.s_BOUNDS[1]):
                            new_v = self.VEL_BOUND_PCT*self.s_BOUNDS[1]
                    
                    new_vel.append(new_v)
                    i += 1
                                                        
                # Calculate new positions
                i = 0
                for p in pos:
                    new_p = p + new_vel[i]
                    
                    # Bound checking
                    # For the network weights
                    if(i>=0 and i<20):
                        if(new_p<self.WEIGHT_BOUNDS[0]):
                            new_p = self.WEIGHT_BOUNDS[0]
                        if(new_p>self.WEIGHT_BOUNDS[1]):
                            new_p = self.WEIGHT_BOUNDS[1]
                    # For the other constants - Tr, Ta, b and s
                    elif(i==20):
                        if(new_p<self.Tr_BOUNDS[0]):
                            new_p = self.Tr_BOUNDS[0]
                        if(new_p>self.Tr_BOUNDS[1]):
                            new_p = self.Tr_BOUNDS[1]                    
                    elif(i==21):
                        if(new_p<self.Ta_BOUNDS[0]):
                            new_p = self.Ta_BOUNDS[0]
                        if(new_p>self.Ta_BOUNDS[1]):
                            new_p = self.Ta_BOUNDS[1]                    
                    elif(i==22):
                        if(new_p<self.b_BOUNDS[0]):
                            new_p = self.b_BOUNDS[0]
                        if(new_p>self.b_BOUNDS[1]):
                            new_p = self.b_BOUNDS[1]                    
                    elif(i==23):
                        if(new_p<self.s_BOUNDS[0]):
                            new_p = self.s_BOUNDS[0]
                        if(new_p>self.s_BOUNDS[1]):
                            new_p = self.s_BOUNDS[1]
                            
                    new_pos.append(new_p)
                    i += 1                    
                            
                # Set the new values
                individual[0:24] = new_pos[0:24]    # 24 element vector
                individual[24:48] = new_vel[0:24]   # 24 element vector
                
                self.population[index] = individual
                
                index += 1
                
            iteration += 1
            
            # Print the results
            print "Best fitness: " + str(self.GBest[24]) + " at " + str(self.GBest[0:24])

if __name__ == '__main__':

        Darwin_Walk_Optimization()

