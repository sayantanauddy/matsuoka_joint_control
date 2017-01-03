function [ osc_name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time ] = fetch_osc_params_from_indiv( osc_name, indiv, osc_index )
%FETCH_OSC_PARAMS_FROM_INDIV Summary of this function goes here
%   Detailed explanation goes here
    disp((osc_index-1)*9 + 1);
    T = indiv((osc_index-1)*9 + 1);
    beta = indiv((osc_index-1) + 1);
    eta = indiv((osc_index-1) + 1);
    sigma = indiv((osc_index-1)*9 + 1);
    theta_star = indiv((osc_index-1)*9 + 1);
    u_i = indiv((osc_index-1)*9 + 1);
    u_j = indiv((osc_index-1)*9 + 1);
    h_psi = indiv((osc_index-1)*9 + 1);
    I = indiv((osc_index-1)*9 + 1);
    gamma = indiv((osc_index-1)*9 + 1);
    start_time = 0.0;

end

