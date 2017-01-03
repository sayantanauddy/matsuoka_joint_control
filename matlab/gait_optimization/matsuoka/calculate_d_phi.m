function [ d_phi_i, d_phi_j ] = calculate_d_phi( oscillator )
%CALCULATE_D_PHI Summary of this function goes here
%   Detailed explanation goes here
    d_phi_i = (-oscillator.phi_i + max([0, oscillator.psi_i]))*1/oscillator.t2;
    d_phi_j = (-oscillator.phi_j + max([0, oscillator.psi_j]))*1/oscillator.t2;
end

