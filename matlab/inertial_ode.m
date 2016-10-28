function dy= inertial_ode( t, y )
%INERTIAL_ODE Summary of this function goes here
%   Detailed explanation goes here
    gamma = 0.5;
    psi = 0.3*sin(t);
    I = 0.08;
    dy = [y(2); -(gamma*y(2) + psi)/I];
end

