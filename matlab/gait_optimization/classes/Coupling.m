classdef Coupling
    %COUPLING Class for a coupling between two different oscillators
    %   Detailed explanation goes here
    
    properties
        mu; % Constant of homologous coupling
        nu; % Constant of non-homologous coupling
        coupled_with; % Identifier of oscillator to which coupling is made
    end
    
    methods
        % Constructor
        function obj = Coupling(mu_val, nu_val, coupled_with_val)
            obj.mu = mu_val;
            obj.nu = nu_val;
            obj.coupled_with = coupled_with_val;
        end
    end
    
end

