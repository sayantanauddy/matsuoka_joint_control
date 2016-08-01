#!/usr/bin/env python

###############################################################################
# File: matsuoka_4_neuron_model.py
# Description: Generates plots for 4 coupled matsuoka neural oscillators
#              As per Network VI in the paper:
#              Matsuoka, Kiyotoshi. "Mechanisms of frequency and pattern control
#              in the neural rhythm generators."
#              Biological cybernetics 56.5-6 (1987): 345-353.
# Execution: python matsuoka_4_neuron_model.py
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
import math

# Declare lists to be used for plots
y1_list = []
y2_list = []
y3_list = []
y4_list = []
y_list = []
t_list = []

# Tunable parameters, static for now
a13 = 1.5
a12 = 1.5
a31 = 1.5
a34 = 1.5
a43 = 1.5
a42 = 1.5
a24 = 1.5
a21 = 1.5

s = 2.0
b = 5.0
Tr = 0.75
Ta = 1.0

# Iteration constants
step = 0.2
count = 200

# Function to calculate next states
def matsuoka(state):
    x1 = state[0]
    x2 = state[1]
    x3 = state[2]
    x4 = state[3]
    f1 = state[4]
    f2 = state[5]
    f3 = state[6]
    f4 = state[7]
    y1 = state[8]
    y2 = state[9]
    y3 = state[10]
    y4 = state[11]

    print "[[" + str(x1) + ", " + str(x2) + ", " + str(x3) + ", " + str(x4) + ", " + \
          str(f1) + ", " + str(f2) + ", " + str(f3) + ", " + str(f4) + ", " + \
          str(y1) + ", " + str(y2) + ", " + str(y3) + ", " + str(y4) + "]]"

    '''
    x1_d = ((-1.0 * a * y2) - (a * y3) - (a * y4) + (s) - (b * f1) - (x1)) / float(Tr)
    x2_d = ((-1.0 * a * y1) - (a * y3) - (a * y4) + (s) - (b * f2) - (x2)) / float(Tr)
    x3_d = ((-1.0 * a * y1) - (a * y2) - (a * y4) + (s) - (b * f3) - (x3)) / float(Tr)
    x4_d = ((-1.0 * a * y2) - (a * y3) - (a * y1) + (s) - (b * f4) - (x4)) / float(Tr)
    '''
    x1_d = ((-1.0 * a21 * y2) - (a31 * y3) + (s) - (b * f1) - (x1)) / float(Tr)
    x2_d = ((-1.0 * a12 * y1) - (a42 * y4) + (s) - (b * f2) - (x2)) / float(Tr)
    x3_d = ((-1.0 * a13 * y1) - (a43 * y4) + (s) - (b * f3) - (x3)) / float(Tr)
    x4_d = ((-1.0 * a24 * y2) - (a34 * y3) + (s) - (b * f4) - (x4)) / float(Tr)

    x1 += step * x1_d
    x2 += step * x2_d
    x3 += step * x3_d
    x4 += step * x4_d

    y1 = max(0.0, x1)
    y2 = max(0.0, x2)
    y3 = max(0.0, x3)
    y4 = max(0.0, x4)

    f1_d = (y1 - f1) / float(Ta)
    f2_d = (y2 - f2) / float(Ta)
    f3_d = (y3 - f3) / float(Ta)
    f4_d = (y4 - f4) / float(Ta)

    f1 += step * f1_d
    f2 += step * f2_d
    f3 += step * f3_d
    f4 += step * f4_d

    return [x1, x2, x3, x4, f1, f2, f3, f4, y1, y2, y3, y4]


# Set initial state
state = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

i = 0
while (i < count):
    state = matsuoka(state)
    y1_list.append(state[8]-state[10])
    y2_list.append(state[9]-state[11])
    #y3_list.append(state[10])
    #y4_list.append(state[11])

    t_list.append(i * step)
    i = i + 1

# Plot
plt.figure(1)
#plt.subplot(411)
plt.plot(t_list, y1_list, label="y12")
#plt.subplot(412)
plt.plot(t_list, y2_list, label="y34")
#plt.subplot(413)
#plt.plot(t_list, y3_list, label="y3")
#plt.subplot(414)
#plt.plot(t_list, y4_list, label="y4")
plt.legend()
# plt.plot(t_list,y2_list)
plt.show()

