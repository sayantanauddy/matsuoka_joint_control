% Script to compute the behavior of a single joint driven by a set of
% neural Matsuoka oscillators

clearvars;
clearvars -global;

% Oscillator constants for oscillator 1
global t1;
global t2;
global beta;
global eta;
global sigma;
global mu;
global nu;
global h_psi;
t1 = 0.3;
t2 = 2.5*t1;
beta = 2.5;
eta = 2.5;
sigma = 1.5;
mu = 0.75;
nu = 0.49;
h_psi = 5.0;

% State variables of oscillator 1
global psi_l_i;
global psi_l_j;
global phi_l_i;
global phi_l_j;
global u_l_i;
global u_l_j;
psi_l_i = 0.0;
psi_l_j = 1.0;
phi_l_i = 0.0;
phi_l_j = 1.0;
u_l_i = 1;
u_l_j = 1;

% State variables of oscillator 2
global psi_r_i;
global psi_r_j;
global phi_r_i;
global phi_r_j;
global u_r_i;
global u_r_j;
psi_r_i = 0.0;
psi_r_j = 1.0;
phi_r_i = 0.0;
phi_r_j = 1.0;
u_r_i = 1;
u_r_j = 1;

% Mean position of oscillation of the joint angle
global theta_star;
theta_star = 0.0;

% Desired time period of joint oscillations
global T_joint;
T_joint= 2;

% Variables required to calculate the time delta
global time_prev;
global time_now;
time_prev = 0;
time_now = 0;

% List used for plotting the torques
global torque_list_l;
torque_list_l = [];
global torque_list_r;
torque_list_r = [];

% List for average position
global avg_position_l;
avg_position_l = [];
global avg_position_r;
avg_position_r = [];

% List for ut
global ut_list_l;
ut_list_l = [];
global ut_list_r;
ut_list_r = [];

% Set the time steps
t = [0:0.01:60];

% Solve the differential equations of the inertial mechanical system
% Use custom ODE solver which uses fixed time steps
% Matlab's in-built ODE solver uses variable time steps and sometimes goes
% back in time to adjust the time step. This causes problems in the state
% variables of the matsuoka oscillator.
y=ode1(@inertial_ode,t,[0.0; 0.0; 0.0; 0.0]);

% Plot the results

hold on;
plot(1);
p1 = plot(t,y(:,1),'b');
p2 = plot(t,y(:,3),'r');
p1(1).LineWidth = 2;
p2(1).LineWidth = 2;
xlabel('time (s)');
ylabel('joint angle (rad)');
set(gca,'fontsize',20);
legend([p1,p2],'left joint angle', 'right joint angle');
%grid on;
%grid minor;



