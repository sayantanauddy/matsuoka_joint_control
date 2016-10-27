function setGlobal()
%SETGLOBAL Summary of this function goes here
%   Detailed explanation goes here

    global psi_i;
    global psi_j;
    global phi_i;
    global phi_j;
    global u_i;
    global u_j;
    
    global time_prev;
    time_prev = 0;
    global time_now;
    time_now = 0;
    
    psi_i = 0.0;
    psi_j = 1.0;
    phi_i = 0.0;
    phi_j = 1.0;
    u_i = 1;
    u_j = 1;

end

