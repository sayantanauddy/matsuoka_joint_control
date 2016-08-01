#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math

# Declare lists to be used for plots
yl_list = []
yr_list = []
y1_list = []
y2_list = []
y3_list = []
y4_list = []
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

    x1 = state['x'][0]
    x2 = state['x'][1]
    x3 = state['x'][2]
    x4 = state['x'][3]

    f1 = state['f'][0]
    f2 = state['f'][1]
    f3 = state['f'][2]
    f4 = state['f'][3]

    y1 = state['y'][0]
    y2 = state['y'][1]
    y3 = state['y'][2]
    y4 = state['y'][3]

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

    ret_state = {}
    ret_state['x'] = [0.0, 0.0, 0.0, 0.0]
    ret_state['f'] = [0.0, 0.0, 0.0, 0.0]
    ret_state['y'] = [0.0, 0.0, 0.0, 0.0]

    ret_state['x'][0] = x1
    ret_state['x'][1] = x2
    ret_state['x'][2] = x3
    ret_state['x'][3] = x4
    ret_state['f'][0] = f1
    ret_state['f'][1] = f2
    ret_state['f'][2] = f3
    ret_state['f'][3] = f4
    ret_state['y'][0] = y1
    ret_state['y'][1] = y2
    ret_state['y'][2] = y3
    ret_state['y'][3] = y4

    return ret_state


# Set initial state
state = {}
state['x'] = [0.0, 0.0, 0.0, 0.0]
state['f'] = [1.0, 0.0, 0.0, 1.0]
state['y'] = [0.0, 0.0, 0.0, 0.0]

i = 0
while (i < count):
    state = matsuoka(state)
    yr_list.append(state['y'][0]-state['y'][1])
    yl_list.append(state['y'][2]-state['y'][3])
    y1_list.append(state['y'][0])
    y2_list.append(state['y'][1])
    y3_list.append(state['y'][2])
    y4_list.append(state['y'][3])
    #y3_list.append(state[10])
    #y4_list.append(state[11])

    t_list.append(i * step)
    i = i + 1

# Plot
plt.figure(1)
plt.subplot(611)
plt.plot(t_list, y1_list, label="y1")
plt.subplot(612)
plt.plot(t_list, y2_list, label="y2")
plt.subplot(613)
plt.plot(t_list, y3_list, label="y3")
plt.subplot(614)
plt.plot(t_list, y4_list, label="y4")
plt.subplot(615)
plt.plot(t_list, yl_list, label="yl")
plt.subplot(616)
plt.plot(t_list, yr_list, label="yr")
plt.legend()
# plt.plot(t_list,y2_list)
plt.show()

