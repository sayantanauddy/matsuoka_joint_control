function dy= inertial_ode( t, y)
%INERTIAL_ODE Summary of this function goes here
%   Detailed explanation goes here
    %y(1) => joint angle of l1
    %y(2) => joint velocity of l1
    %y(3) => joint angle of l2
    %y(4) => joint velocity of l2
    %y(5) => joint angle of l3
    %y(6) => joint velocity of l3
    %y(7) => joint angle of l4
    %y(8) => joint velocity of l4
    %y(9) => joint angle of l5
    %y(10) => joint velocity of l5
    %y(11) => joint angle of r1
    %y(12) => joint velocity of r1
    %y(13) => joint angle of r2
    %y(14) => joint velocity of r2
    %y(15) => joint angle of r3
    %y(16) => joint velocity of r3
    %y(17) => joint angle of r4
    %y(18) => joint velocity of r4
    %y(19) => joint angle of r5
    %y(20) => joint velocity of r5


    % Global oscillators
    global l1 l2 l3 l4 l5 r1 r2 r3 r4 r5;

    % Pass the current time and joint angle to the oscillator function
    [l1.psi,l2.psi,l3.psi,l4.psi,l5.psi,r1.psi,r2.psi,r3.psi,r4.psi,r5.psi] = ...
    matsuoka_torque(t,y(1),y(3),y(5),y(7),y(9),y(11),y(13),y(15),y(17),y(19)); 
        
    % Starting oscillators at different times introduces an initial phase shift that affects
    % the final phase relationships between oscillators

    if t<l1.start_time
        l1.psi = 0.0;
    end
    if t<l2.start_time
        l2.psi = 0.0;
    end
    if t<l3.start_time
        l3.psi = 0.0;
    end
    if t<l4.start_time
        l4.psi = 0.0;
    end
    if t<l5.start_time
        l5.psi = 0.0;
    end
    if t<r1.start_time
        r1.psi = 0.0;
    end
    if t<r2.start_time
        r2.psi = 0.0;
    end
    if t<r3.start_time
        r3.psi = 0.0;
    end
    if t<r4.start_time
        r4.psi = 0.0;
    end
    if t<r5.start_time
        r5.psi = 0.0;
    end

    dy = [y(2);  -(l1.gamma*y(2)  + l1.psi)/l1.I; ...
          y(4);  -(l2.gamma*y(4)  + l2.psi)/l2.I; ...
          y(6);  -(l3.gamma*y(6)  + l3.psi)/l3.I; ...
          y(8);  -(l4.gamma*y(8)  + l4.psi)/l4.I; ...
          y(10); -(l5.gamma*y(10) + l5.psi)/l5.I; ...
          y(12); -(r1.gamma*y(12) + r1.psi)/r1.I; ...
          y(14); -(r2.gamma*y(14) + r2.psi)/r2.I; ...
          y(16); -(r3.gamma*y(16) + r3.psi)/r3.I; ...
          y(18); -(r4.gamma*y(18) + r4.psi)/r4.I; ...
          y(20); -(r5.gamma*y(20) + r5.psi)/r5.I];
end
