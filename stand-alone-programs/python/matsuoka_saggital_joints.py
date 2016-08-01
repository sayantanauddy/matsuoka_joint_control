#!/usr/bin/env python

###############################################################################
# File: matsuoka_saggital_joints.py
# Description: Generates plots for a coupled matsuoka neural oscillator network
# Execution: python matsuoka_saggital_joints.py
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
import math

step = 0.2
count = 500

# Function to calculate the next state of the Matsuoka neural oscillators
def matsuoka(individual, state):
    print "State: " + str(state)
    print "Individual: " + str(individual)

    # Extract the weights
    w1 = individual[0]
    w2 = individual[1]
    w3 = individual[2]
    w4 = individual[3]
    w5 = individual[4]
    w6 = individual[5]
    w7 = individual[6]
    w8 = individual[7]
    w9 = individual[8]
    w10 = individual[9]
    w11 = individual[10]
    w12 = individual[11]
    w13 = individual[12]
    w14 = individual[13]
    w15 = individual[14]
    w16 = individual[15]
    w17 = individual[16]
    w18 = individual[17]
    w19 = individual[18]
    w20 = individual[19]

    # Extract the network constants
    Tr = 1.75 #individual[20]
    Ta = 3.0# individual[21]
    b = 5.0 #individual[22]
    s = 10.0 #individual[23]

    # Extract the current states
    # The numbering scheme: For f21 <variable=f><oscillator#=2><1 for extensor, 2 for flexor>
    # Each oscillator has the state [x1,x2,f1,f2,y1,y2]
    # There are 6 such oscillators
    x11 = state[0]
    x12 = state[1]
    f11 = state[2]
    f12 = state[3]
    y11 = state[4]
    y12 = state[5]

    x21 = state[6]
    x22 = state[7]
    f21 = state[8]
    f22 = state[9]
    y21 = state[10]
    y22 = state[11]

    x31 = state[12]
    x32 = state[13]
    f31 = state[14]
    f32 = state[15]
    y31 = state[16]
    y32 = state[17]

    x41 = state[18]
    x42 = state[19]
    f41 = state[20]
    f42 = state[21]
    y41 = state[22]
    y42 = state[23]

    x51 = state[24]
    x52 = state[25]
    f51 = state[26]
    f52 = state[27]
    y51 = state[28]
    y52 = state[29]

    x61 = state[30]
    x62 = state[31]
    f61 = state[32]
    f62 = state[33]
    y61 = state[34]
    y62 = state[35]

    # Calculate the derivatives of the membrane potentials
    # This is based on the connections between the individual neurons
    x11_d = ((-1.0 * w1 * y12) - (w6 * y21) + (s) - (b * f11) - (x11)) / float(Tr)
    x12_d = ((-1.0 * w2 * y11) - (w8 * y22) + (s) - (b * f12) - (x12)) / float(Tr)

    x21_d = ((-1.0 * w3 * y22) - (w5 * y11) + (s) - (b * f21) - (x21)) / float(Tr)
    x22_d = ((-1.0 * w4 * y21) - (w7 * y12) + (s) - (b * f22) - (x22)) / float(Tr)

    x31_d = ((-1.0 * w13 * y32) - (w9 * y11) + (s) - (b * f31) - (x31)) / float(Tr)
    x32_d = ((-1.0 * w14 * y31) + (s) - (b * f32) - (x32)) / float(Tr)

    x41_d = ((-1.0 * w15 * y42) - (w10 * y21) + (s) - (b * f41) - (x41)) / float(Tr)
    x42_d = ((-1.0 * w16 * y41) + (s) - (b * f42) - (x42)) / float(Tr)

    x51_d = ((-1.0 * w17 * y52) + (s) - (b * f51) - (x51)) / float(Tr)
    x52_d = ((-1.0 * w18 * y51) - (w11 * y11) + (s) - (b * f52) - (x52)) / float(Tr)

    x61_d = ((-1.0 * w19 * y62) + (s) - (b * f61) - (x61)) / float(Tr)
    x62_d = ((-1.0 * w20 * y61) - (w12 * y21) + (s) - (b * f62) - (x62)) / float(Tr)

    # Calculate the elements of the next state
    x11 = x11 + step * x11_d
    x12 = x12 + step * x12_d
    x21 = x21 + step * x21_d
    x22 = x22 + step * x22_d
    x31 = x31 + step * x31_d
    x32 = x32 + step * x32_d
    x41 = x41 + step * x41_d
    x42 = x42 + step * x42_d
    x51 = x51 + step * x51_d
    x52 = x52 + step * x52_d
    x61 = x61 + step * x61_d
    x62 = x62 + step * x62_d

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

    f11_d = (y11 - f11) / float(Ta)
    f12_d = (y12 - f12) / float(Ta)
    f21_d = (y21 - f21) / float(Ta)
    f22_d = (y22 - f22) / float(Ta)
    f31_d = (y31 - f31) / float(Ta)
    f32_d = (y32 - f32) / float(Ta)
    f41_d = (y41 - f41) / float(Ta)
    f42_d = (y42 - f42) / float(Ta)
    f51_d = (y51 - f51) / float(Ta)
    f52_d = (y52 - f52) / float(Ta)
    f61_d = (y61 - f61) / float(Ta)
    f62_d = (y62 - f62) / float(Ta)

    f11 = f11 + step * f11_d
    f12 = f12 + step * f12_d
    f21 = f21 + step * f21_d
    f22 = f22 + step * f22_d
    f31 = f31 + step * f31_d
    f32 = f32 + step * f32_d
    f41 = f41 + step * f41_d
    f42 = f42 + step * f42_d
    f51 = f51 + step * f51_d
    f52 = f52 + step * f52_d
    f61 = f61 + step * f61_d
    f62 = f62 + step * f62_d

    # Create the return state
    ret_state = [x11, x12, f11, f12, y11, y12,
                 x21, x22, f21, f22, y21, y22,
                 x31, x32, f31, f32, y31, y32,
                 x41, x42, f41, f42, y41, y42,
                 x51, x52, f51, f52, y51, y52,
                 x61, x62, f61, f62, y61, y62
                 ]

    return ret_state


y1_list = []
y2_list = []
y3_list = []
y4_list = []
y5_list = []
y6_list = []
t_list = []
# Set initial state
state = []
ind = 0
while (ind < 6):
    state.extend([0.0, 1.0, 0.0, 1.0, 0.0, 0.0])
    ind += 1

individual = [3.0, 2.9157543434582105, 2.96242964129441, 2.359770940893763, 2.787845063918853, 2.6903966359947953, 2.454181783625989, 3.0, 2.6511088621108163, 1.7574080781081307, 1.852836778530851, 2.4737328871765785, 1.7604025623886785, 1.6572799282935364, 1.8944508834600342, 2.575886107138607, 2.042106149143515, 2.4699122609783495, 2.9913573767339363, 2.5046608394675927, 0.16438996534182976, 0.75, 1.63081399890768, 7.502218557248396]
i=0
while(i<count):
    state = matsuoka(individual, state)
    y1_list.append((state[0*6 + 4] - state[0*6 + 5]))
    y2_list.append((state[1*6 + 4] - state[1*6 + 5]))
    y3_list.append((state[2*6 + 4] - state[2*6 + 5]))
    y4_list.append((state[3*6 + 4] - state[3*6 + 5]))
    y5_list.append((state[4*6 + 4] - state[4*6 + 5]))
    y6_list.append((state[5*6 + 4] - state[5*6 + 5]))
    t_list.append(i*step)
    i=i+1

# Plot
plt.figure()
plt.plot(t_list, y1_list)
plt.plot(t_list, y2_list)
plt.plot(t_list, y3_list)
plt.plot(t_list, y4_list)
plt.plot(t_list, y5_list)
plt.plot(t_list, y6_list)
plt.show()