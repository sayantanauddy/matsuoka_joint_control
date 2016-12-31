function [ d_psi_i, d_psi_j ] = calculate_differential( oscillator_name, oscillator, current_angle_map, theta_star_map )
%CALCULATE_DIFFERENTIAL Summary of this function goes here
%   Detailed explanation goes here

    coupling_keys = keys(oscillator.couplings);
        
    sum_i = -oscillator.psi_i ...
            -(oscillator.beta*oscillator.phi_i) ...                                                                  % Self inhibition
            -oscillator.eta*max([0,oscillator.psi_j]) ...                                                            % Mutual inhibition
            -oscillator.sigma*max([0,  -(current_angle_map{oscillator_name} - theta_star_map{oscillator_name})]);	 % Sensory feedback from own angle
      
    for i = 1:length(oscillator.couplings)
        sum_i = sum_i - oscillator.couplings(coupling_keys{i}).mu * max([0, -(current_angle_map{i} - theta_star_map{i})])...
                      - oscillator.couplings(coupling_keys{i}).nu * max([0, (current_angle_map{i} - theta_star_map{i})]);
    end
    
    d_psi_i = (sum_i + oscillator.u_i)/oscillator.t1;
        
    sum_j = -oscillator.psi_j ...
           -(oscillator.beta*oscillator.phi_j) ...                                                                  % Self inhibition
           -oscillator.eta*max([0,oscillator.psi_i]) ...                                                            % Mutual inhibition
           -oscillator.sigma*max([0, (current_angle_map{oscillator_name} - theta_star_map{oscillator_name})]);	    % Sensory feedback from own angle
      
    for i = 1:length(oscillator.couplings)
        sum_j = sum_j - oscillator.couplings(coupling_keys{i}).mu * max([0, (current_angle_map{i} - theta_star_map{i})])...
                      - oscillator.couplings(coupling_keys{i}).nu * max([0, -(current_angle_map{i} - theta_star_map{i})]);
    end
    
    d_psi_j = (sum_j + oscillator.u_j)/oscillator.t1;
    
end

