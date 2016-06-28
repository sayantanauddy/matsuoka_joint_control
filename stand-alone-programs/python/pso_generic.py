#!/usr/bin/env python

###############################################################################
# File: pso_generic.py
# Description: Particle Swarm Optimization of the Eggholder function (2D vector)
# Execution: python pso_generic.py
###############################################################################


import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time


def fitness(x):
    x1 = x[0]
    x2 = x[1]
    
    # The Eggholder function
    y = -1.0*(x2+47.0)*math.sin(math.sqrt(abs(x2 + (x1/2.0) + 47.0))) - x1*math.sin(math.sqrt(abs(x1 - (x2+47.0))))
    return y
    
        
def pso():

    # Environment constants
    ENV_UPPER_BOUND = 512.0
    ENV_LOWER_BOUND = -512.0

    # PSO specific constants
    POPULATION_SIZE = 4000
    MAX_ITERS = 1000
    C1 = 2.0 # Acceleration constant
    C2 = 2.0 # Acceleration constant
    V_MAX = 100.0 # Bounded velocity in each dimension
    
    # PSO specific variables
    gbest_fitness = 0.0
    gbest_x1 = 0.0
    gbest_x2 = 0.0    
    w = 0.9 # Inertial weight, linearly scales from 0.9 to 0.4 over the entire run
    # Each individual state is a 7 element vector - [x1, x2, v1, v2, pbest_x1, pbest_x2, pbest_fitness]
    # The population is stored in a list of such vectors
    population = []

    # Initialize the population    
    index = 0
    while(index<POPULATION_SIZE):
        x1 = random.uniform(ENV_LOWER_BOUND, ENV_UPPER_BOUND)
        x2 = random.uniform(ENV_LOWER_BOUND, ENV_UPPER_BOUND)
        v1 = random.uniform(-1.0*V_MAX, V_MAX)
        v2 = random.uniform(-1.0*V_MAX, V_MAX)
        population.append([x1,x2,v1,v2,x1,x2,0.0])
        index += 1
        
    iteration = 0
    while(iteration < MAX_ITERS):
        
        # Set the inertial weight
        w = 0.9 - (iteration*(0.9 - 0.4)/MAX_ITERS)

        index = 0
        for individual in population:
        
            # Unpack the vector
            x1 = individual[0]
            x2 = individual[1]
            v1 = individual[2]
            v2 = individual[3]
            pbest_fitness = individual[6]
            
            # Calculate fitness of individual
            indiv_fitness = fitness([x1,x2])
            
            # Check pbest
            if(indiv_fitness<pbest_fitness):
                pbest_fitness = indiv_fitness
                pbest_x1 = x1
                pbest_x2 = x2
                population[index] = [x1,x2,v1,v2,pbest_x1,pbest_x2,pbest_fitness]
                
            # Check gbest
            if(indiv_fitness<gbest_fitness):
                gbest_fitness = indiv_fitness
                gbest_x1 = x1
                gbest_x2 = x2
            
            index += 1
            
        # Once pbest and gbest have been determined, x and v need to be updated
        index = 0
        for indiv in population: 

            # Unpack the vector
            x1 = individual[0]
            x2 = individual[1]
            v1 = individual[2]
            v2 = individual[3]
            pbest_x1 = individual[4]
            pbest_x2 = individual[5]         
            pbest_fitness = individual[6]               
        
            # Generate 2 random numbers
            rand1 = random.random()
            rand2 = random.random()
            
            # Calculate new velocity
            new_v1 = w*v1 + C1*rand1*(pbest_x1-x1) + C2*rand2*(gbest_x1-x1)
            new_v2 = w*v2 + C1*rand1*(pbest_x2-x2) + C2*rand2*(gbest_x2-x2)
            
            if(new_v1>V_MAX):
                new_v1 = V_MAX
            
            if(new_v2>V_MAX):
                new_v2 = V_MAX
            
            # Calculate new positions
            new_x1 = x1 + new_v1
            new_x2 = x2 + new_v2
            
            # Bound the positions
            if(new_x1>ENV_UPPER_BOUND):
                new_x1 = ENV_UPPER_BOUND
            if(new_x1<ENV_LOWER_BOUND):
                new_x1 = ENV_LOWER_BOUND
            if(new_x2>ENV_UPPER_BOUND):
                new_x2 = ENV_UPPER_BOUND
            if(new_x2<ENV_LOWER_BOUND):
                new_x2 = ENV_LOWER_BOUND
                        
            # Set the new values
            population[index] = [new_x1,new_x2,new_v1,new_v2,pbest_x1,pbest_x2,pbest_fitness]
            
            index += 1
            
        iteration += 1
        
    # Print the results
    print "Best fitness: " + str(gbest_fitness) + " at " + str([gbest_x1,gbest_x2])
    
pso()    
        

