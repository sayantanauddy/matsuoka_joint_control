function [ q_out ] = modpi( q_in )
%MODPI Summary of this function goes here
%   Detailed explanation goes here
    full = 2*pi;
    q_mod = mod(q_in,2*full);
    
    if q_mod <= full
        q_out = q_mod;
    elseif q_mod > full & q_mod <= 2*full
        q_out = -(2*full-q_mod);
        
end

