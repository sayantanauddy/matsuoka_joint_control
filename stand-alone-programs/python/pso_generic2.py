# -*- coding: utf-8 -*-
"""
Created on Tue Dec  6 15:51:43 2016

@author: sayantan
"""
import numpy as np
import random
import math

# Initialize a population
x_bounds = [[0.0,1.0],[0.0,1.0],[0.0,1.0],[0.0,1.0],[0.0,1.0],[0.0,1.0]]

# PSO specific constants
POPULATION_SIZE = 40
MAX_ITERS = 1000
C1 = 2.0 # Acceleration constant
C2 = 2.0 # Acceleration constant

def eggholder_2d(x):
    x1 = x[0]
    x2 = x[1]
    
    # The Eggholder function
    y = -1.0*(x2+47.0)*math.sin(math.sqrt(abs(x2 + (x1/2.0) + 47.0))) - x1*math.sin(math.sqrt(abs(x1 - (x2+47.0))))
    return y

def hartmann_6d(x):
    
    alpha = np.reshape(np.array([1.0, 1.2, 3.0, 3.2]),(4,1));
    
    A = np.array([[10, 3, 17, 3.50, 1.7, 8],
    [0.05, 10, 17, 0.1, 8, 14],
    [3, 3.5, 1.7, 10, 17, 8],
    [17, 8, 0.05, 10, 0.1, 14]])
    
    P = (10**(-4)) * np.array([[1312, 1696, 5569, 124, 8283, 5886],
    [2329, 4135, 8307, 3736, 1004, 9991],
    [2348, 1451, 3522, 2883, 3047, 6650],
    [4047, 8828, 8732, 5743, 1091, 381]])
    
    outer = 0.0
    for i in range(0,4):
        inner = 0.0
        for j in range(0,6):
            inner += A[i,j]*((x[j]-P[i][j])**2)
        outer += alpha[i]*math.exp(-1.0*inner)
    outer = -(1/1.94)*(2.58+outer)
    return outer[0]
    
    
def pso():

    population_size = POPULATION_SIZE
    
    # Determine the length of the individual from the length of x_bounds
    individual_length = len(x_bounds)
    
    # Initialize the population
    population = []
    
    g_best_position = []
    g_best_fitness = np.inf
    
    for i in range(0,population_size):
        
        # Initialize an individual
        x = np.zeros(individual_length)
        v = np.zeros(individual_length)
        p_best_position = np.zeros(individual_length)
        p_best_fitness = np.Inf
                
        for j in range(0,individual_length):
            # Initialize the position as a random number between the upper and lower bounds
            x[j] = random.uniform(x_bounds[j][0],x_bounds[j][1])
            
            # Initialize the velocity according to SPSO-07
            v[j] = 0.5*(random.uniform(x_bounds[j][0],x_bounds[j][1]) - random.uniform(x_bounds[j][0],x_bounds[j][1]))
            
        
        individual = [x, v, p_best_position, p_best_fitness]
        # Add the individual to the population
        population.append(individual)
        
    for iteration in range(0, MAX_ITERS):
        
        # Set the inertial weight
        w = 0.9 - (iteration*(0.9 - 0.6)/MAX_ITERS)
        
        
        for index in range(0,len(population)):
            
            individual = population[index]
            
            # Unpack the individual
            x = individual[0]
            v = individual[1]
            p_best_position = individual[2]
            p_best_fitness = individual[3]
            
            # Calculate fitness of individual
            indiv_fitness = hartmann_6d(x)
            
             # Check pbest
            if(indiv_fitness<p_best_fitness):
                p_best_position = x
                p_best_fitness = indiv_fitness
                individual = [x, v, p_best_position, p_best_fitness]
                population[index] = individual
                
            # Check gbest
            if(indiv_fitness<g_best_fitness):
                g_best_fitness = indiv_fitness
                g_best_position = x
                print "Current best fitness: " + str(g_best_fitness)
                
        # Once pbest and gbest have been determined, x and v need to be updated
        for index in range(0,len(population)):
            
            individual = population[index]
            
            # Unpack the individual
            x = individual[0]
            v = individual[1]
            p_best_position = individual[2]
            p_best_fitness = individual[3]
            
            # Generate 2 random numbers
            rand1 = random.random()
            rand2 = random.random()
            
            # Calculate new velocity and position
            for vel_index in range(0,len(v)):
                
                
                old_vel = v[vel_index]
                new_vel = w*old_vel + C1*rand1*(p_best_position[vel_index] - x[vel_index]) \
                +  C2*rand2*(g_best_position[vel_index] - x[vel_index])  
                
            
                # Check the velocity bounds
                if new_vel<-0.001:
                    new_vel = -0.001
                if new_vel>0.001:
                    new_vel = 0.001
                
                
                v[vel_index] = new_vel

                # Calculate new positions
                new_pos = x[vel_index] + new_vel

                # Check the position bounds
                if new_pos<x_bounds[vel_index][0]:
                    new_pos = x_bounds[vel_index][0]
                if new_pos>x_bounds[vel_index][1]:
                    new_pos = x_bounds[vel_index][1]
                    
                x[vel_index] = new_pos
                
                
            # Set the new values
            population[index] = [x, v, p_best_position, p_best_fitness]
                
    # Print the results
    print "Best fitness: " + str(g_best_fitness) + " at " + str(g_best_position)
    
pso()    
                