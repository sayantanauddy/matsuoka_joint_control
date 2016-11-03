% Script to compute the behavior of a single joint driven by a set of
% neural Matsuoka oscillators

clearvars;
clearvars -global;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setup the python module
if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end
mod = py.importlib.import_module('nao');
py.reload(mod);

py.nao.setJointAngle('HeadYaw',0,1.0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State variables of the oscillator
global psi_i;
global psi_j;
global phi_i;
global phi_j;
global u_i;
global u_j;

% Mean position of oscillation of the joint angle
global theta_star;

% Variables required to calculate the time delta
global time_prev;
global time_now;

% List used for plotting the torques
global torque_list;

theta_star = 0.0;
torque_list = [];
time_prev = 0;
time_now = 0;
psi_i = 0.0;
psi_j = 1.0;
phi_i = 0.0;
phi_j = 1.0;
u_i = 1;%-3.6337e-05 %2;
u_j = 1;%-3.6337e-05 %2;

% Set the time steps
t = [0:0.05:20];

% Solve the differential equations of the inertial mechanical system
% Use custom ODE solver which uses fixed time steps
% Matlab's in-built ODE solver uses variable time steps and sometimes goes
% back in time to adjust the time step. This causes problems in the state
% variables of the matsuoka oscillator.
y=ode1(@inertial_ode,t,[0; 0]);

% Plot the results
hold on;
plot(t,y(:,1));
plot(t,torque_list);
legend('joint angle','torque');
xlabel('time');
ylabel('torque/angle');
hold off;

