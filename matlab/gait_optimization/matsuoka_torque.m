function [l1_psi,l2_psi,l3_psi,l4_psi,l5_psi,r1_psi,r2_psi,r3_psi,r4_psi,r5_psi] = matsuoka_torque(t, q_l1, q_l2, q_l3, q_l4, q_l5, q_r1, q_r2, q_r3, q_r4, q_r5)	
%   Function to compute the desired torque at each time step
%   using the equations for the matsuoka osccillator
	% Global oscillators

    global l1 l2 l3 l4 l5 r1 r2 r3 r4 r5;

    % Variables required to calculate the time delta
    global time_prev;
    global time_now;

    % Calculate the time delta
    time_now = t;
    step_time = time_now - time_prev;
    time_prev = time_now;

    disp(t);

    % At t=10s switch on the mutual coupling between the 2 oscillators - TO DO


    % Repititive code. Find a way to do this using a function
    % create a keystore of q_r1...
    % create a keystore of r1.theta_star....
    % Then use the coupling key to access
    current_angle_map_keys = {'l1','l2','l3','l4','l5','r1','r2','r3','r4','r5'};
    current_angle_map_values = [q_l1, q_l2, q_l3, q_l4, q_l5, q_r1, q_r2, q_r3, q_r4, q_r5];
    current_angle_map = containers.Map(current_angle_map_keys,current_angle_map_values);
    theta_star_map_keys = {'l1','l2','l3','l4','l5','r1','r2','r3','r4','r5'};
    theta_star_map_values = [l1.theta_star, l2.theta_star, l3.theta_star, l4.theta_star, l5.theta_star, ....
                             r1.theta_star, r2.theta_star, r3.theta_star, r4.theta_star, r5.theta_star];
    theta_star_map = containers.Map(theta_star_map_keys,theta_star_map_values);

    % Calculate the derivative of the discharge rate
    [ l1.d_psi_i, l1.d_psi_j ] = calculate_d_psi('l1', l1, current_angle_map, theta_star_map);
    [ l2.d_psi_i, l2.d_psi_j ] = calculate_d_psi('l2', l2, current_angle_map, theta_star_map);
    [ l3.d_psi_i, l3.d_psi_j ] = calculate_d_psi('l3', l3, current_angle_map, theta_star_map);
    [ l4.d_psi_i, l4.d_psi_j ] = calculate_d_psi('l4', l4, current_angle_map, theta_star_map);
    [ l5.d_psi_i, l5.d_psi_j ] = calculate_d_psi('l5', l5, current_angle_map, theta_star_map);
    [ r1.d_psi_i, r1.d_psi_j ] = calculate_d_psi('r1', r1, current_angle_map, theta_star_map);
    [ r2.d_psi_i, r2.d_psi_j ] = calculate_d_psi('r2', r2, current_angle_map, theta_star_map);
    [ r3.d_psi_i, r3.d_psi_j ] = calculate_d_psi('r3', r3, current_angle_map, theta_star_map);
    [ r4.d_psi_i, r4.d_psi_j ] = calculate_d_psi('r4', r4, current_angle_map, theta_star_map);
    [ r5.d_psi_i, r5.d_psi_j ] = calculate_d_psi('r5', r5, current_angle_map, theta_star_map);

    % Calculate the next values of the discharge rate
    l1.psi_i = l1.psi_i + l1.d_psi_i* step_time;
    l1.psi_j = l1.psi_j + l1.d_psi_j* step_time;
    l2.psi_i = l2.psi_i + l2.d_psi_i* step_time;
    l2.psi_j = l2.psi_j + l2.d_psi_j* step_time;
    l3.psi_i = l3.psi_i + l3.d_psi_i* step_time;
    l3.psi_j = l3.psi_j + l3.d_psi_j* step_time;
    l4.psi_i = l4.psi_i + l4.d_psi_i* step_time;
    l4.psi_j = l4.psi_j + l4.d_psi_j* step_time;
    l5.psi_i = l5.psi_i + l5.d_psi_i* step_time;
    l5.psi_j = l5.psi_j + l5.d_psi_j* step_time;
    r1.psi_i = r1.psi_i + r1.d_psi_i* step_time;
    r1.psi_j = r1.psi_j + r1.d_psi_j* step_time;
    r2.psi_i = r2.psi_i + r2.d_psi_i* step_time;
    r2.psi_j = r2.psi_j + r2.d_psi_j* step_time;
    r3.psi_i = r3.psi_i + r3.d_psi_i* step_time;
    r3.psi_j = r3.psi_j + r3.d_psi_j* step_time;
    r4.psi_i = r4.psi_i + r4.d_psi_i* step_time;
    r4.psi_j = r4.psi_j + r4.d_psi_j* step_time;
    r5.psi_i = r5.psi_i + r5.d_psi_i* step_time;
    r5.psi_j = r5.psi_j + r5.d_psi_j* step_time;

    % Calculate the derivative of the state variable
    [ l1.d_phi_i, l1.d_phi_j ] = calculate_d_phi(l1);
    [ l2.d_phi_i, l2.d_phi_j ] = calculate_d_phi(l2);
    [ l3.d_phi_i, l3.d_phi_j ] = calculate_d_phi(l3);
    [ l4.d_phi_i, l4.d_phi_j ] = calculate_d_phi(l4);
    [ l5.d_phi_i, l5.d_phi_j ] = calculate_d_phi(l5);
    [ r1.d_phi_i, r1.d_phi_j ] = calculate_d_phi(r1);
    [ r2.d_phi_i, r2.d_phi_j ] = calculate_d_phi(r2);
    [ r3.d_phi_i, r3.d_phi_j ] = calculate_d_phi(r3);
    [ r4.d_phi_i, r4.d_phi_j ] = calculate_d_phi(r4);
    [ r5.d_phi_i, r5.d_phi_j ] = calculate_d_phi(r5);

    % Calculate the next values of the state variable
    l1.phi_i = l1.phi_i + l1.d_phi_i*step_time;
    l1.phi_j = l1.phi_j + l1.d_phi_j*step_time;
    l2.phi_i = l2.phi_i + l2.d_phi_i*step_time;
    l2.phi_j = l2.phi_j + l2.d_phi_j*step_time;
    l3.phi_i = l3.phi_i + l3.d_phi_i*step_time;
    l3.phi_j = l3.phi_j + l3.d_phi_j*step_time;
    l4.phi_i = l4.phi_i + l4.d_phi_i*step_time;
    l4.phi_j = l4.phi_j + l4.d_phi_j*step_time;
    l5.phi_i = l5.phi_i + l5.d_phi_i*step_time;
    l5.phi_j = l5.phi_j + l5.d_phi_j*step_time;
    r1.phi_i = r1.phi_i + r1.d_phi_i*step_time;
    r1.phi_j = r1.phi_j + r1.d_phi_j*step_time;
    r2.phi_i = r2.phi_i + r2.d_phi_i*step_time;
    r2.phi_j = r2.phi_j + r2.d_phi_j*step_time;
    r3.phi_i = r3.phi_i + r3.d_phi_i*step_time;
    r3.phi_j = r3.phi_j + r3.d_phi_j*step_time;
    r4.phi_i = r4.phi_i + r4.d_phi_i*step_time;
    r4.phi_j = r4.phi_j + r4.d_phi_j*step_time;
    r5.phi_i = r5.phi_i + r5.d_phi_i*step_time;
    r5.phi_j = r5.phi_j + r5.d_phi_j*step_time;

    % Calculate the output torque
    l1.psi = l1.h_psi*(max([0, l1.psi_i]) - max([0, l1.psi_j]));
    l2.psi = l2.h_psi*(max([0, l2.psi_i]) - max([0, l2.psi_j]));
    l3.psi = l3.h_psi*(max([0, l3.psi_i]) - max([0, l3.psi_j]));
    l4.psi = l4.h_psi*(max([0, l4.psi_i]) - max([0, l4.psi_j]));
    l5.psi = l5.h_psi*(max([0, l5.psi_i]) - max([0, l5.psi_j]));
    r1.psi = r1.h_psi*(max([0, r1.psi_i]) - max([0, r1.psi_j]));
    r2.psi = r2.h_psi*(max([0, r2.psi_i]) - max([0, r2.psi_j]));
    r3.psi = r3.h_psi*(max([0, r3.psi_i]) - max([0, r3.psi_j]));
    r4.psi = r4.h_psi*(max([0, r4.psi_i]) - max([0, r4.psi_j]));
    r5.psi = r5.h_psi*(max([0, r5.psi_i]) - max([0, r5.psi_j]));

    l1_psi = l1.psi;
	l2_psi = l2.psi;
	l3_psi = l3.psi;
	l4_psi = l4.psi;
	l5_psi = l5.psi;
	r1_psi = r1.psi;
	r2_psi = r2.psi;
	r3_psi = r3.psi;
	r4_psi = r4.psi;
	r5_psi = r5.psi;

end

