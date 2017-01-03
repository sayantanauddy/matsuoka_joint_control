
% Global oscillators
global l1 l2 l3 l4 l5 r1 r2 r3 r4 r5;

% Variables required to calculate the time delta
global time_prev time_now;
time_prev = 0;
time_now = 0;

% Variable which controls when the oscillator couplings are switched on
global coupling_start_time coupling_on;
coupling_start_time = 5.0;
coupling_on = false;

% Initialize oscillators of left leg
% Oscillator(T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time)
l1 = Oscillator('l1',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
l2 = Oscillator('l2',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
l3 = Oscillator('l3',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
l4 = Oscillator('l4',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
l5 = Oscillator('l5',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);

% Initialize the oscillators of the right leg
% Oscillator(T,beta,eta,sigma,theta_star,start_time,u_i,u_j,h_psi,I,gamma)
r1 = Oscillator('r1',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
r2 = Oscillator('r2',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
r3 = Oscillator('r3',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
r4 = Oscillator('r4',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);
r5 = Oscillator('r5',1.5,2.5,2.5,0.75,0.0,1.0,1.0,5.0,0.08,0.5,0.0);

% Set the oscillator couplings
% Create Coupling objects, put them in a hash table accessed by the name of the 
% which the current oscillator is coupled with
% Coupling(mu,nu,coupled_with)

l1.couplings = containers.Map;
l1.couplings('r1') = Coupling(0.75,0.49,'r1');
l1.couplings('l2') = Coupling(0.75,0.49,'l2');
l1.couplings('l4') = Coupling(0.75,0.49,'l4');

l2.couplings = containers.Map;
l2.couplings('l1') = Coupling(0.75,0.49,'l1');
l2.couplings('l3') = Coupling(0.75,0.49,'l3');

l3.couplings = containers.Map;
l3.couplings('l2') = Coupling(0.75,0.49,'l2');

l4.couplings = containers.Map;
l4.couplings('l1') = Coupling(0.75,0.49,'l1');
l4.couplings('l5') = Coupling(0.75,0.49,'l5');

l5.couplings = containers.Map;
l5.couplings('l4') = Coupling(0.75,0.49,'l4');

r1.couplings = containers.Map;
r1.couplings('l1') = Coupling(0.75,0.49,'l1');
r1.couplings('r2') = Coupling(0.75,0.49,'r2');
r1.couplings('r4') = Coupling(0.75,0.49,'r4');

r2.couplings = containers.Map;
r2.couplings('r1') = Coupling(0.75,0.49,'r1');
r2.couplings('r3') = Coupling(0.75,0.49,'r3');

r3.couplings = containers.Map;
r3.couplings('r2') = Coupling(0.75,0.49,'r2');

r4.couplings = containers.Map;
r4.couplings('r1') = Coupling(0.75,0.49,'r1');
r4.couplings('r5') = Coupling(0.75,0.49,'r5');

r5.couplings = containers.Map;
r5.couplings('r4') = Coupling(0.75,0.49,'r4');

% Set the time steps
t = [0:0.05:30];

% Use custom ODE solver which uses fixed time steps
% Matlab's in-built ODE solver uses variable time steps and sometimes goes
% back in time to adjust the time step. This causes problems in the state
% variables of the matsuoka oscillator.
y=ode1(@inertial_ode,t,[ 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 ]);

% Plot the results
hold on;
plot(1);
p1 = plot(t,y(:,1));
%p3 = plot(t,y(:,3));
%p5 = plot(t,y(:,5));
%p7 = plot(t,y(:,7));
%p9 = plot(t,y(:,9));
p11 = plot(t,y(:,11));
%p13 = plot(t,y(:,13));
%p15 = plot(t,y(:,15));
%p17 = plot(t,y(:,17));
%p19 = plot(t,y(:,19));
p1(1).LineWidth = 2;
%p3(1).LineWidth = 2;
%p5(1).LineWidth = 2;
%p7(1).LineWidth = 2;
%p9(1).LineWidth = 2;
p11(1).LineWidth = 2;
%p13(1).LineWidth = 2;
%p15(1).LineWidth = 2;
%p17(1).LineWidth = 2;
%p19(1).LineWidth = 2;
xlabel('time (s)');
ylabel('joint angle (rad)');
hold off;
legend([p1,p11],'l1','r1');
set(gca,'fontsize',20);

