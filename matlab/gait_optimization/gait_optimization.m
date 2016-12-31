
% Global oscillators
global l1 l2 l3 l4 l5 r1 r2 r3 r4 r5;

% Global list of oscillators
global oscillator_list_obj;

% Variables required to calculate the time delta
global time_prev;
global time_now;
time_prev = 0;
time_now = 0;

% Initialize oscillators of left leg
% Oscillator(T,beta,eta,sigma,theta_star,start_time,u_i,u_j,h_psi,I,gamma)
l1 = Oscillator('l1',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
l2 = Oscillator('l2',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
l3 = Oscillator('l3',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
l4 = Oscillator('l4',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
l5 = Oscillator('l5',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);

% Initialize the oscillators of the right leg
% Oscillator(T,beta,eta,sigma,theta_star,start_time,u_i,u_j,h_psi,I,gamma)
r1 = Oscillator('r1',2.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
r2 = Oscillator('r2',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
r3 = Oscillator('r3',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
r4 = Oscillator('r4',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);
r5 = Oscillator('r5',1.5,2.5,2.5,1.5,0.0,0.0,1.0,1.0,5.0,0.08,0.5);

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
l4.couplings('l1') = Coupling(0.75,0.49,'l1'),
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
r4.couplings('r1') = Coupling(0.75,0.49,'r1'),
r4.couplings('r5') = Coupling(0.75,0.49,'r5');

r5.couplings = containers.Map;
r5.couplings('r4') = Coupling(0.75,0.49,'r4');

% Set the time steps
t = [0:0.01:30];

