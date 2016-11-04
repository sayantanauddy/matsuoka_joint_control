function dy= inertial_ode( t, y)%, ft, tors )
%INERTIAL_ODE Summary of this function goes here
%   Detailed explanation goes here
    %y(1) => joint angle of l
    %y(2) => joint velocity of l
    %y(3) => joint angle of r
    %y(4) => joint velocity of r
    gamma = 0.5;
    % Pass the current time and joint angle to the oscillator function
    [psi_l, psi_r] = matsuoka_torque(t,y(1),y(3)); 
    I = 0.08;
    dy = [y(2); -(gamma*y(2) + psi_l)/I; y(4); -(gamma*y(4) + psi_r)/I];
end

