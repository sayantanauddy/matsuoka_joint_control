function [ l1, l2, l3, l4, l5, r1, r2, r3, r4, r5, coupling_start_time, joint_start_time ] = extract_network( indiv )
%EXTRACT_NETWORK Creates a network of oscillators from the individual
%vector. The network configuration is controlled by this function
%   Detailed explanation goes here
    
    % Variable which controls when the oscillator couplings are switched on
    coupling_start_time = indiv(64);
    
    % Time at which the robot joints are activated
    joint_start_time = indiv(65);

    % Construct the oscillator network from the indiv vector
    
    % Initialize oscillators of left leg
    % Oscillator(name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time)
    % The value for start_time returned here is 0.0 and will be set later on
    % Initialize the oscillators of the right leg by cloning those of the left leg
    [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( 'l1', indiv, 1 );
    l1 = Oscillator(osc_name,T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);
    r1 = Oscillator('r1',T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);

    [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( 'l2', indiv, 2 );
    l2 = Oscillator(osc_name,T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);    
    r2 = Oscillator('r2',T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);    
    
    [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( 'l3', indiv, 3 );
    l3 = Oscillator(osc_name,T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);
    r3 = Oscillator('r3',T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);
    
    [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( 'l4', indiv, 4 );
    l4 = Oscillator(osc_name,T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);   
    r4 = Oscillator('r4',T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);   

    [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( 'l5', indiv, 5 );
    l5 = Oscillator(osc_name,T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);    
    r5 = Oscillator('r5',T,beta,eta,sigma,0.0,u_i,u_j,h_psi,I,gamma,start_time);    
    
    % Set the oscillator start_time(s)
    l1.start_time = indiv(58);
    r1.start_time = indiv(59);
    l2.start_time = l1.start_time + indiv(60);
    r2.start_time = r1.start_time + indiv(60);
    l3.start_time = l1.start_time + indiv(61);
    r3.start_time = r1.start_time + indiv(61);
    l4.start_time = l1.start_time + indiv(62);
    r4.start_time = r1.start_time + indiv(62);    
    l5.start_time = l1.start_time + indiv(63);
    r5.start_time = r1.start_time + indiv(63);    
    
    
    % Extract the coupling values from indiv vector (sent by pso)
    % coupling osc1-osc2 = coupling osc3-osc4 implies that 
    % (mu,nu) for osc1-osc2 = (mu,nu) for osc3-osc4
    
    % coupling r1-l1 = coupling l1-r1
    mu_r1_l1 = indiv(46);
    nu_r1_l1 = indiv(47);
    mu_l1_r1 = mu_r1_l1;
    nu_l1_r1 = nu_r1_l1;
    
    % coupling r1-r2 = coupling l1-l2
    mu_r1_r2 = indiv(48);
    nu_r1_r2 = indiv(49);
    mu_l1_l2 = mu_r1_r2;
    nu_l1_l2 = nu_r1_r2;
    
    % coupling r2-r1 = coupling l2-l1
    mu_r2_r1 = indiv(50);
    nu_r2_r1 = indiv(51);
    mu_l2_l1 = mu_r2_r1;
    nu_l2_l1 = nu_r2_r1;
    
    % coupling r2-r3 = coupling r3-r2 = coupling l2-l3 = coupling l3-l2
    mu_r2_r3 = indiv(52);
    nu_r2_r3 = indiv(53);
    mu_r3_r2 = mu_r2_r3;
    nu_r3_r2 = nu_r2_r3;
    mu_l2_l3 = mu_r2_r3;
    nu_l2_l3 = nu_r2_r3;
    mu_l3_l2 = mu_r2_r3;
    nu_l3_l2 = nu_r2_r3;
 
    % coupling r1-r4 = coupling r4-r1 = coupling l1-l4 = coupling l4-l1
    mu_r1_r4 = indiv(54);
    nu_r1_r4 = indiv(55);
    mu_r4_r1 = mu_r1_r4;
    nu_r4_r1 = nu_r1_r4;
    mu_l1_l4 = mu_r1_r4;
    nu_l1_l4 = nu_r1_r4;
    mu_l4_l1 = mu_r1_r4;
    nu_l4_l1 = nu_r1_r4;
    
    % coupling r4-r5 = coupling r5-r4 = coupling l4-l5 = coupling l5-l4
    mu_r4_r5 = indiv(56);
    nu_r4_r5 = indiv(57); 
    mu_r5_r4 = mu_r4_r5;
    nu_r5_r4 = nu_r4_r5;
    mu_l4_l5 = mu_r4_r5;
    nu_l4_l5 = nu_r4_r5;
    mu_l5_l4 = mu_r4_r5;
    nu_l5_l4 = nu_r4_r5;
    
    % Set the oscillator couplings
    % Create Coupling objects, put them in a hash table accessed by the name of the 
    % which the current oscillator is coupled with
    % Coupling(mu,nu,coupled_with)
    
    l1.couplings = containers.Map;
    l1.couplings('r1') = Coupling(mu_l1_r1,nu_l1_r1,'r1');
    l1.couplings('l2') = Coupling(mu_l1_l2,nu_l1_l2,'l2');
    l1.couplings('l4') = Coupling(mu_l1_l4,nu_l1_l4,'l4');

    l2.couplings = containers.Map;
    l2.couplings('l1') = Coupling(mu_l2_l1,nu_l2_l1,'l1');
    l2.couplings('l3') = Coupling(mu_l2_l3,nu_l2_l3,'l3');

    l3.couplings = containers.Map;
    l3.couplings('l2') = Coupling(mu_l3_l2,nu_l3_l2,'l2');

    l4.couplings = containers.Map;
    l4.couplings('l1') = Coupling(mu_l4_l1,nu_l4_l1,'l1');
    l4.couplings('l5') = Coupling(mu_l4_l5,nu_l4_l5,'l5');

    l5.couplings = containers.Map;
    l5.couplings('l4') = Coupling(mu_l5_l4,nu_l5_l4,'l4');

    r1.couplings = containers.Map;
    r1.couplings('l1') = Coupling(mu_r1_l1,nu_r1_l1,'l1');
    r1.couplings('r2') = Coupling(mu_r1_r2,nu_r1_r2,'r2');
    r1.couplings('r4') = Coupling(mu_r1_r4,nu_r1_r4,'r4');

    r2.couplings = containers.Map;
    r2.couplings('r1') = Coupling(mu_r2_r1,nu_r2_r1,'r1');
    r2.couplings('r3') = Coupling(mu_r2_r3,nu_r2_r3,'r3');

    r3.couplings = containers.Map;
    r3.couplings('r2') = Coupling(mu_r3_r2,nu_r3_r2,'r2');

    r4.couplings = containers.Map;
    r4.couplings('r1') = Coupling(mu_r4_r1,nu_r4_r1,'r1');
    r4.couplings('r5') = Coupling(mu_r4_r5,nu_r4_r5,'r5');

    r5.couplings = containers.Map;
    r5.couplings('r4') = Coupling(mu_r5_r4,nu_r5_r4,'r4');

end

