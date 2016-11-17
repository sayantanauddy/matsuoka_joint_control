function [psi_t_l,psi_t_r] = matsuoka_torque(t, q_l, q_r)
% 
%   Function to compute the desired torque at each time step
%   using the equations for the matsuoka osccillator

    % Oscillator constants
    global t1_l;
    global t2_l;
    global t1_r;
    global t2_r;
    global beta;
    global eta;
    global sigma;
    global mu;
    global nu;  
    global h_psi;
   
    % State variables of the oscillator
    % Implemented as global variables so that these persist between
    % invocations
    global psi_l_i;
    global psi_l_j;
    global phi_l_i;
    global phi_l_j;
    global u_l_i;
    global u_l_j;
    
    global psi_r_i;
    global psi_r_j;
    global phi_r_i;
    global phi_r_j;
    global u_r_i;
    global u_r_j;

    global time_now;
    global time_prev;
    global torque_list_l;
    global torque_list_r;
    global avg_position_l;
    global avg_position_r;
    global ut_list_l;
    global ut_list_r;
    
    global jointname_l;
    global jointname_r;
    
    jointname_l = 'LShoulderPitch';
    jointname_r = 'RShoulderPitch';
    
    % Mean position of oscillation of the joint angle
    global theta_star;
    
    
    u_t = 1;
    u_i= u_t;
    u_j = u_t;
    
    % Calculate the time delta
    time_now = t;
    step_time = time_now - time_prev;
    time_prev = time_now;
    
    if step_time>0
        py.nao.setJointAngle(jointname_l, jointname_r, radtodeg(q_l), radtodeg(q_r), step_time);
        disp(t);
        pause(0.01)
    end
    
    % At t=10s switch on the mutual coupling between the 2 oscillators
    if t<10
        mu = 0;
        nu = 0;
        sigma = 1.5;
    else
        mu = 0.75;
        nu = 0.49;  
        sigma = 0.75;
    end

    % Calculate the proprioceptive feedback
    %ref_l = q_l - theta_star;
    %ref_r = q_r - theta_star;
    % Get the sensed angle
    pySensedAngle_l = py.nao.getJointAngle(jointname_l);
    pySensedAngle_r = py.nao.getJointAngle(jointname_r);

    % Convert to Matlab type
    cp_l = cell(pySensedAngle_l);
    cp_r = cell(pySensedAngle_r);
    matlabSensedAngle_l = cellfun(@double,cp_l);
    matlabSensedAngle_r = cellfun(@double,cp_r);
    
    % The oscillator differential equations
    dpsi_l_i = (-psi_l_i -(beta*phi_l_i) -eta*max([0,psi_l_j])  -sigma*max([0, -(matlabSensedAngle_l - theta_star)]) - 0.5*mu*max([0, -(matlabSensedAngle_r - theta_star)]) - 0.5*nu*max([0, (matlabSensedAngle_r - theta_star)]) + 0.5*u_i)*1/t1_l;
    dpsi_l_j = (-psi_l_j -(beta*phi_l_j) -eta*max([0,psi_l_i])  -sigma*max([0, (matlabSensedAngle_l - theta_star)])  - 0.5*mu*max([0, (matlabSensedAngle_r - theta_star)]) - 0.5*nu*max([0, -(matlabSensedAngle_r - theta_star)]) + 0.5*u_j)*1/t1_l;

    dpsi_r_i = (-psi_r_i -(beta*phi_r_i) -eta*max([0,psi_r_j])  -sigma*max([0, -(matlabSensedAngle_r - theta_star)]) - mu*max([0, -(matlabSensedAngle_l - theta_star)]) - nu*max([0, (matlabSensedAngle_l - theta_star)]) + u_i)*1/t1_r;
    dpsi_r_j = (-psi_r_j -(beta*phi_r_j) -eta*max([0,psi_r_i])  -sigma*max([0, (matlabSensedAngle_r - theta_star)])  - mu*max([0, (matlabSensedAngle_l - theta_star)]) - nu*max([0, -(matlabSensedAngle_l - theta_star)]) + u_j)*1/t1_r;

    % Calculate next state
    psi_l_i = psi_l_i + dpsi_l_i* step_time;
    psi_l_j = psi_l_j + dpsi_l_j* step_time;
    
    psi_r_i = psi_r_i + dpsi_r_i* step_time;
    psi_r_j = psi_r_j + dpsi_r_j* step_time;

    % Calculate derivative of the state variable
    dphi_l_i = (-phi_l_i + max([0, psi_l_i]))*1/t2_l;
    dphi_l_j = (-phi_l_j + max([0, psi_l_j]))*1/t2_l;
    
    dphi_r_i = (-phi_r_i + max([0, psi_r_i]))*1/t2_r;
    dphi_r_j = (-phi_r_j + max([0, psi_r_j]))*1/t2_r;
    
    % Calculate the next state
    phi_l_i = phi_l_i + dphi_l_i*step_time;
    phi_l_j = phi_l_j + dphi_l_j*step_time;
    
    phi_r_i = phi_r_i + dphi_r_i*step_time;
    phi_r_j = phi_r_j + dphi_r_j*step_time;

    % Calculate the output torque
    psi_t_l = h_psi*(max([0, psi_l_i]) - max([0, psi_l_j]));
    psi_t_r = h_psi*(max([0, psi_r_i]) - max([0, psi_r_j]));
    
    % List used for plotting the torques
    torque_list_l = [torque_list_l psi_t_l];
    torque_list_r = [torque_list_r psi_t_r];
    
    %List for average position
    avg_position_l = [avg_position_l theta_star];
    
    % List for ut
    ut_list_l = [ut_list_l u_i]; %u_i=u_j=u_t
    
end

