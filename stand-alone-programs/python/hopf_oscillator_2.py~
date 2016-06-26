import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import odeint
from scipy.interpolate import interp1d
from scipy.interpolate import spline


Q_learned = 0.0
error_graph = []
tt = []

def P_teach(t):
    return 0.8*math.sin(15.0*t)+math.cos(30.0*t)-1.4*math.sin(45.0*t)-0.5*math.cos(60.0*t)

def calc_derivatives(state, theta0, omega0,F_t):

    # constants
    mu = 1.0
    gamma = 8.0
    epsilon = 0.9
    eta = 0.5
    tau = 2.0
        
    # Unpack the state vector
    x = state[0]
    y = state[1]
    omega = state[2]
    alpha = state[3]
    phi = state[4]
    
    r = math.sqrt(x**2 + y**2)
    theta = np.sign(x)*math.acos(-1.0*y/r)
    
    # Calculate the derivatives
    x_d = (gamma*(mu - (r**2))*x) - (omega*y) + (epsilon*F_t) + (tau*math.sin(theta-phi))
    y_d = gamma*(mu - (r**2))*y + (omega*x)
    omega_d = (-1.0*F_t*y)/r
    alpha_d = eta*x*F_t
    phi_d = math.sin(((omega/omega0)*theta0) - theta - phi)
    
    return [x_d, y_d, omega_d, alpha_d, phi_d]
    
        
def hopf(state, t):

    global Q_learned
    global error_graph
    
    # Hopf oscillator 0    
    x0 = state[0]
    y0 = state[1]
    omega0 = state[2]
    alpha0 = state[3]
    phi0 = state[4]
    
    r0 = math.sqrt(x0**2 + y0**2)
    theta0 = np.sign(x0)*math.acos(-1.0*y0/r0)
    
    # Hopf oscillator 1 
    x1 = state[5]
    y1 = state[6]
    omega1 = state[7]
    alpha1 = state[8]
    phi1 = state[9]

    # Hopf oscillator 2
    x2 = state[10]
    y2 = state[11]
    omega2 = state[12]
    alpha2 = state[13]
    phi2 = state[14]
    
    # Hopf oscillator 3 
    x3 = state[15]
    y3 = state[16]
    omega3 = state[17]
    alpha3 = state[18]
    phi3 = state[19]

    
    # Calculate the learned signal
    Q_learned = alpha0*x0 + alpha1*x1 + alpha2*x2 + alpha3*x3
    
    # Calculate the signal difference
    F_t = P_teach(t) - Q_learned 
    
    # Calculate the error
    error = F_t**2
    error_graph.append(error)
    tt.append(t)
    print 'Error at time ' + str(t) + '=' + str(error)
    
    # Calculate derivatives for oscillator 0
    [x0_d, y0_d, omega0_d, alpha0_d, phi0_d] = calc_derivatives([x0, y0, omega0, alpha0, phi0], theta0, omega0, F_t)
    
    # Calculate derivatives for oscillator 1    
    [x1_d, y1_d, omega1_d, alpha1_d, phi1_d] = calc_derivatives([x1, y1, omega1, alpha1, phi1], theta0, omega0, F_t)
    
    # Calculate derivatives for oscillator 2    
    [x2_d, y2_d, omega2_d, alpha2_d, phi2_d] = calc_derivatives([x2, y2, omega2, alpha2, phi2], theta0, omega0, F_t)
    
    # Calculate derivatives for oscillator 3    
    [x3_d, y3_d, omega3_d, alpha3_d, phi3_d] = calc_derivatives([x3, y3, omega3, alpha3, phi3], theta0, omega0, F_t)
    
    # Construct the return state vector
    return_state = [x0_d, y0_d, omega0_d, alpha0_d, phi0_d, x1_d, y1_d, omega1_d, alpha1_d, phi1_d, x2_d, y2_d, omega2_d, alpha2_d, phi2_d, x3_d, y3_d, omega3_d, alpha3_d, phi3_d]
    return return_state


# Create a uniform distribution for initial omegas
omega_zeros = np.linspace(6.0,70.0,4)


# Create the initial state
# [x0, y0, omega0, alpha0, phi0, x1, y1, omega1, alpha1, phi1, x2, y2, omega2, alpha2, phi2, x3, y3, omega3, alpha3, phi3]
state0 = [1.0,0.0,omega_zeros[0],0.0,0.0, 1.0,0.0,omega_zeros[1],0.0,0.0, 1.0,0.0,omega_zeros[2],0.0,0.0, 1.0,0.0,omega_zeros[3],0.0,0.0]

time = np.arange(0.0,1300.0,0.1)
state = odeint(hopf,state0,time)

'''
plt.figure()
x_sm = np.array(time)
y_sm = np.array(state[:,0])
err = np.array(error_graph)
ttt = np.array(tt)
print np.shape(x_sm)
print np.shape(err)
print np.shape(state[:,0])

x_smooth = np.linspace(x_sm.min(), x_sm.max(), 100)
y_smooth = spline(time, y_sm, x_smooth)

# Calculate the teaching signal
P_teach_list = []
for t in time:
    P_teach_list.append(P_teach(t))
P_teach_arr = np.array(P_teach_list)
P_teach_smooth = spline(time, P_teach_arr, x_smooth)


plt.plot(x_smooth, y_smooth, 'red', linewidth=1)
plt.plot(x_smooth, P_teach_smooth, 'blue', linewidth=1)
plt.plot(ttt, err, 'green', linewidth=1)
plt.ylim([-10.0,40.0])
plt.xlabel('Time')
plt.legend(('x','P_teach'))
plt.title('Hopf oscillator equations')
plt.show()
'''





