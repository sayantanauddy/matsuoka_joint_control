
% Function to compute the desired torque at each time step
% using the equations for the matsuoka osccillator
function psi_t = matsuoka_torque(~, t, q, qd)
    t1 = 0.05;
    t2 = 0.125;
    beta = 2.5;
    eta = 2.5;
    sigma = 1.5;
    h_psi = 5;
    theta_star = 0.0;
    
    global psi_i;
    global psi_j;
    global phi_i;
    global phi_j;
    global u_i;
    global u_j;
    global time_prev;
    global time_now;
    
    time_now = t;
    step_time = time_now - time_prev;
    time_prev = time_now;
    
    disp(time_now);
    
    dpsi_i = (-psi_i -(beta*phi_i) -(eta*max([0,psi_j]))  - sigma*(q(1) - theta_star) + u_i)*1/t1;
    dpsi_j = (-psi_j -(beta*phi_j) -(eta*max([0,psi_i]))  - sigma*(-(q(1) - theta_star))+ u_j)*1/t1;
    
    % Calculate next state
    psi_i = psi_i + dpsi_i* step_time;
    psi_j = psi_j + dpsi_j* step_time;
    
    % Calculate derivative of the state variable
    dphi_i = (-phi_i + max([0, psi_i]))*1/t2;
    dphi_j = (-phi_j + max([0, psi_j]))*1/t2;
    
    % Calculate the next state
    phi_i = phi_i + dphi_i*step_time;
    phi_j = phi_j + dphi_j*step_time;
    
    % Calculate the output torque
    psi_1 = h_psi*(max([0, psi_i]) - max([0, psi_j]));
    
    psi_t = [psi_1 0];
    
end

