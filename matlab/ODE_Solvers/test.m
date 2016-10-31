%initialize
global psi_i;
global psi_j;

global phi_i;
global phi_j;

global u_i;
global u_j;

global time_prev;
global time_now;
global time_now_list;
global torque_list;

time_now_list = [];
torque_list = [];

time_prev = 0;
time_now = 0;
psi_i = 0.0;
psi_j = 1.0;
phi_i = 0.0;
phi_j = 1.0;
u_i = 1;
u_j = 1;

%generate torque
ft = [0:0.01:30]; % Generate t for f 
tors = arrayfun(@(x) matsuoka_torque(x,0), ft);

[T, Y] = ode45(@(t,y) inertial_ode(t, y, ft, tors), ft,  [0, 0]);

hold on;
plot(T,Y);
plot(ft, tors);