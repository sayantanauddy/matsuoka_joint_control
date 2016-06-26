#!/usr/bin/env python

###############################################################################
# File: matsuoka.py
# Description: Generates plots for a coupled matsuoka neural oscillator
# Execution: python matsuoka.py
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
import math

# Declare lists to be used for plots
y1_list = []
y2_list = []
t_list = []

# Tunable parameters, static for now
a  = 1.5
s  = 5.0
b  = 2.5
Tr = 1.0
Ta = 15.0

# Iteration constants
step = 0.2
count = 500
    
# Function to calculate next states
def matsuoka(state):

    x1 = state[0]
    x2 = state[1]
    f1 = state[2]
    f2 = state[3]
    y1 = state[4]
    y2 = state[5]
    
        
    x1_d = ((-1.0*a*y2) + (s) - (b*f1) - (x1))/float(Tr)
    x2_d = ((-1.0*a*y1) + (s) - (b*f2) - (x2))/float(Tr)    
    
    x1 = x1 + step*x1_d
    x2 = x2 + step*x2_d
    
    y1 = max(0.0, x1)
    y2 = max(0.0, x2)
    
    
    f1_d = (y1 - f1)/float(Ta)
    f2_d = (y2 - f2)/float(Ta)
    
    f1 = f1 + step*f1_d
    f2 = f2 + step*f2_d
    
    return [x1, x2, f1, f2, y1, y2]
    

# Set initial state
state = [0.0, 1.0, 0.0, 1.0, 0.0 ,0.0]

i=0
while(i<count):
    state = matsuoka(state)
    y1_list.append(state[4])
    y2_list.append(state[5])
    t_list.append(i*step)
    i=i+1

# Plot
plt.figure()
plt.plot(t_list,y1_list)
plt.plot(t_list,y2_list)
plt.show()

