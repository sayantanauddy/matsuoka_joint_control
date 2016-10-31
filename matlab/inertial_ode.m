function dy= inertial_ode( t, y)%, ft, tors )
%INERTIAL_ODE Summary of this function goes here
%   Detailed explanation goes here
    gamma = 0.5;
    % Pass the current time and joint angle to the oscillator function
    psi = matsuoka_torque(t,y(1)); 
    I = 0.08;
    dy = [y(2); -(gamma*y(2) + psi)/I];
end

