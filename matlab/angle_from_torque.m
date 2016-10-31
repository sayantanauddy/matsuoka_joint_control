

global psi_i;
global psi_j;
global phi_i;
global phi_j;
global u_i;
global u_j;

global time_prev;
global time_now;
global torque_list;

torque_list = [];

time_prev = 0;
time_now = 0;
psi_i = 0.0;
psi_j = 1.0;
phi_i = 0.0;
phi_j = 1.0;
u_i = 2;
u_j = 2;

t = [0:0.001:20];
y=ode1(@inertial_ode,t,[0; 0]);

hold on;
plot(t,y(:,1));
plot(t,torque_list);
hold off;