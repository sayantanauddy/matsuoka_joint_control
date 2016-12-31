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

    % At t=10s switch on the mutual coupling between the 2 oscillators - TO DO


    % Repititive code. Find a way to do this using a function
    % create a keystore of q_r1...
    % create a keystore of r1.theta_star....
    % Then use the coupling key to access
    current_angle_map_keys = {'l1','l2','l3','l4','l5','r1','r2','r3','r4','r5'};
    current_angle_map_values = [q_l1, q_l2, q_l3, q_l4, q_l5, q_r1, q_r2, q_r3, q_r4, q_r5];
    current_angle_map = containers.Map(current_angle_map_keys,current_angle_map_values);
    theta_star_map_keys = current_angle_map_keys;
    theta_star_map_values = [l1.theta_star, l2.theta_star, l3.theta_star, l4.theta_star, l5.theta_star, ....
                             r1.theta_star, r2.theta_star, r3.theta_star, r4.theta_star, r5.theta_star];
    theta_star_map = containers.Map(theta_star_map_keys,theta_star_map_values);

    % Calculate the derivative of the discharge rate
    [ l1.d_psi_i, l1.d_psi_j ] = calculate_differential('l1', l1, current_angle_map, theta_star_map);
    [ l2.d_psi_i, l2.d_psi_j ] = calculate_differential('l2', l2, current_angle_map, theta_star_map);
    [ l3.d_psi_i, l3.d_psi_j ] = calculate_differential('l3', l3, current_angle_map, theta_star_map);
    [ l4.d_psi_i, l4.d_psi_j ] = calculate_differential('l4', l4, current_angle_map, theta_star_map);
    [ l5.d_psi_i, l5.d_psi_j ] = calculate_differential('l5', l5, current_angle_map, theta_star_map);
    [ r1.d_psi_i, r1.d_psi_j ] = calculate_differential('r1', r1, current_angle_map, theta_star_map);
    [ r2.d_psi_i, r2.d_psi_j ] = calculate_differential('r2', r2, current_angle_map, theta_star_map);
    [ r3.d_psi_i, r3.d_psi_j ] = calculate_differential('r3', r3, current_angle_map, theta_star_map);
    [ r4.d_psi_i, r4.d_psi_j ] = calculate_differential('r4', r4, current_angle_map, theta_star_map);
    [ r5.d_psi_i, r5.d_psi_j ] = calculate_differential('r5', r5, current_angle_map, theta_star_map);

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



end

