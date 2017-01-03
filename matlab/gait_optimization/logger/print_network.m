function [ nw_str ] = print_network( l1, l2, l3, l4, l5, r1, r2, r3, r4, r5, coupling_start_time, joint_start_time )
%PRINT_NETWORK Prints the whole network in a formatted readable way (Use
%sprintf to show the string returned by this function
%   Detailed explanation goes here
    
    nw_str = '############# Printing entire oscillator network #############  \n';
        
    nw_str = strcat(nw_str,...
                    l1.print(), l2.print(), l3.print(), l4.print(), l5.print(), ...
                    r1.print(), r2.print(), r3.print(), r4.print(), r5.print(), ...
                    'coupling_start_time = ', num2str(coupling_start_time), '\n', ...
                    'joint_start_time = ', num2str(joint_start_time), '\n\n',...
                    '############################################################## ');

                
end

