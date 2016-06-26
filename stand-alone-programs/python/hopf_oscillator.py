from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math

def F(t):
    #return 0
    if t>20:
        return 0
    else:
        return math.sin(t)
    
def hopf(state, t):

    # constants
    mu = 1
    gamma = 8
    epsilon = 0.9


    # unpack the state vector
    x = state[0]
    y = state[1]
    omega = state[2]
    
    r = math.sqrt(x*x + y*y)
    
    x_d = gamma*(mu - r*r)*x - omega*y + epsilon*F(t)
    y_d = gamma*(mu - r*r)*y + omega*x
    omega_d = -1*epsilon*F(t)*y/r

    return [x_d, y_d, omega_d]
    

t = np.arange(0,50,0.1)
state0 = [1.0,0.0,1]
state = odeint(hopf,state0,t)
print np.shape(state)

F_list = []
for t_ in t:
    F_list.append(F(t_))

#print state[:,0]
print F_list

plt.figure()
plt.plot(t,state[:,0])
plt.plot(t,F_list)
plt.ylim([-10,10])
plt.xlabel('Time')
plt.legend(('x','F'))
plt.title('Hopf oscillator equations')
plt.show()
