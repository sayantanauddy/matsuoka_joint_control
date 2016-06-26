# Problem with how time is computed
# Need to see details of odeint

from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math

y1 = 0.0
y2 = 0.0

a  = 1.5
s  = 5.0
b  = 2.5
Tr = 1.0
Ta = 12.0

step = 0.2

y1_list = []
y2_list = []

def matsuoka(state, t):

    global y1,y2
    
    print t

    x1 = state[0]
    x2 = state[1]
    f1 = state[2]
    f2 = state[3]
       
    x1_d = ((-1.0*a*y2) + (s) - (b*f1) - (x1))/float(Tr)
    x2_d = ((-1.0*a*y1) + (s) - (b*f2) - (x2))/float(Tr)    
    
    x1 = x1 + step*x1_d
    x2 = x2 + step*x2_d
    
    y1 = max(0.0, x1)
    y2 = max(0.0, x2)
    
    y1_list.append(y1)
    y2_list.append(y2)
    
    f1_d = (y1 - f1)/float(Ta)
    f2_d = (y2 - f2)/float(Ta)
    
    return [x1_d, x2_d, f1_d, f2_d]
    

state0 = [0.0, 0.1, 0.0, 1.0]
t = np.arange(0,5000,step)
state = odeint(matsuoka,state0,t)

y1_arr = np.array(y1_list)
y2_arr = np.array(y2_list)


plt.figure()
plt.plot(t[0:1000],y1_arr[0:1000])
plt.plot(t[0:1000],y2_arr[0:1000])
plt.show()


