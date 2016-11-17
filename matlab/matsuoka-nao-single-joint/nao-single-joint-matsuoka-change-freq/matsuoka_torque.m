function psi_t = matsuoka_torque(t, q)
% 
%   Function to compute the desired torque at each time step
%   using the equations for the matsuoka osccillator

    % Joint name
    global jointname;

    % Oscillator constants
    global t1;
    global t2;
    global beta;
    global eta;
    global sigma;
    global h_psi;
   
    % State variables of the oscillator
    % Implemented as global variables so that these persist between
    % invocations
    global psi_i;
    global psi_j;
    global phi_i;
    global phi_j;
    global u_i;
    global u_j;
    global time_now;
    global time_prev;
    global torque_list;
    global avg_position;
    
    % Mean position of oscillation of the joint angle
    global theta_star;
    
    % Calculate the time delta
    time_now = t;
    step_time = time_now - time_prev;
    time_prev = time_now;

    % The time period of position =1.0 from 0 to 10s, =2.0 from 10 to 20s,
%     % = 3.0 from 20s to 30s
%     if t<=10
%         T = 1.0;
%     elseif t<=20
%         T = 2.0;
%     elseif t<=30;
%         T = 3.0;
%     end
    
    T = 1.0 + (3.0 - 1.0)*t/30.0;
    u_i = 3.5 - (3.5 - 1.5)*t/30.0;
    u_j = u_i;
    
    t1 = 2.13 + 0.6804*T - sqrt(4.512 + 2.685*T);
    t2 = 2.5*t1;
    
    datestr(now,'dd-mm-yyyy HH:MM:SS FFF')
    % Use the incoming q to drive the joint
    if step_time>0
        py.nao.setJointAngle(jointname,radtodeg(q),step_time);
        pause(0.1)
    end
    
    % Get the sensed angle
    pySensedAngle = py.nao.getJointAngle(jointname);

    % Convert to Matlab type
    cp = cell(pySensedAngle);
    matlabSensedAngle = cellfun(@double,cp);
    
    % Calculate the proprioceptive feedback
    ref = matlabSensedAngle - theta_star;
    
    % The oscillator differential equations
    dpsi_i = (-psi_i -(beta*phi_i) -eta*max([0,psi_j])  -sigma*max([0, -ref]) + u_i)*1/t1;
    dpsi_j = (-psi_j -(beta*phi_j) -eta*max([0,psi_i])  -sigma*max([0, ref])+ u_j)*1/t1;

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
    psi_t = h_psi*(max([0, psi_i]) - max([0, psi_j]));
    
    % List used for plotting the torques
    torque_list = [torque_list psi_t];
    
    %List for average position
    avg_position = [avg_position theta_star];
    
end

