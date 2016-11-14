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

% Time after which second oscillator should be switched on. This is used to
% introduce a time lag.
global second_osc_on_time;
second_osc_on_time = 0.0;

% Set the time steps
t = [0:0.01:30];

% Solve the differential equations of the inertial mechanical system
% Use custom ODE solver which uses fixed time steps
% Matlab's in-built ODE solver uses variable time steps and sometimes goes
% back in time to adjust the time step. This causes problems in the state
% variables of the matsuoka oscillator.

T = 0.3;
second_osc_on_time = 0.0;

result = zeros(38,51);
i=1;
j=1;

while T <= 4.0
    while second_osc_on_time <= 5.0
        
        t1 = 2.13 + 0.6804*T - sqrt(4.512 + 2.685*T);
        t2 = 2.5*t1;
        
        psi_l_i = 0.0;
        psi_l_j = 1.0;
        phi_l_i = 0.0;
        phi_l_j = 1.0;
        u_l_i = 1;
        u_l_j = 1;
        psi_r_i = 0.0;
        psi_r_j = 1.0;
        phi_r_i = 0.0;
        phi_r_j = 1.0;
        u_r_i = 1;
        u_r_j = 1;
        time_prev = 0;
        time_now = 0;
        torque_list_l = [];
        torque_list_r = [];
        avg_position_l = [];
        avg_position_r = [];
        ut_list_l = [];
        ut_list_r = [];
        
        y=ode1(@inertial_ode,t,[0.0; 0.0; 0.0; 0.0]);
        
        % Calculate the phase difference
        % [http://stackoverflow.com/questions/27545171/identifying-phase-shift-between-signals]
        y1_h = hilbert(y(3001,1));
        y3_h = hilbert(y(3001,3));
        phase_rad = angle(y1_h ./ y3_h);
        
        result(i,j) = phase_rad;
        disp(i);
        disp(j);
        disp('----');
        second_osc_on_time = second_osc_on_time + 0.1;
        j = j + 1;
        
    end
    T = T + 0.1;
    i = i + 1;
    % Reset variables
    second_osc_on_time = 0.0;
    j = 1;
    disp('=====');
    disp(result);
end

% Plot the results

figure;
hold on;
[xx,yy]=meshgrid(0.3:0.1:4.0,0.0:0.1:5.0);
surf(xx,yy,transpose(result),'EdgeColor','None');
view(2);
xlabel('Time period of joint oscillation (s)');
ylabel('Difference of activation (s)');
set(gca,'fontsize',20);
colormap(bone);
%grid on;
%grid minor;



