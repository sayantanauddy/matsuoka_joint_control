function [t1,t2] = derive_time_constants(T)
%DERIVE_TIME_CONSTANTS Derives the oscillator time constants from the time
%period of joint oscillation
%   Accepts the desired time period of oscillation and returns the two
%   oscillator time constants
    t1 = 2.13 + 0.6804*T - sqrt(4.512 + 2.685*T);
    t2 = 2.5*t1;
end
