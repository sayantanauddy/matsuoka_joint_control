classdef Oscillator
    %Oscillator Class encapsulating the behavior of a single oscillator
    %   Detailed explanation goes here
    
    properties
        name; % Descriptor to identify an oscillator
        
        % Oscillator constants
        T; % Time period of oscillation of joint
        t1; % Time constant of the rate of discharge
        t2; % Time constant of fatigue
        beta; % Self-inhibition constant
        eta; % Constant of mutual inhibition
        sigma; % Strength of sensory feedback
        couplings; % List of Coupling objects (with other oscillators)
        theta_star; % Reference position of oscillation
        start_time; % Time after t=0 when oscillator should be switched on
        u_i; % Tonic input to neuron i
        u_j; % Tonic input to neuron j
        h_psi; % Torque gain factor
        I; % Moment of inertia
        gamma; % Damping constant
        
        % Oscillator variables
        psi_i;
        psi_j;
        phi_i;
        phi_j;
        psi; % Net torque
        theta; % Current angle
        
        % Oscillator derivatives
        d_psi_i;
        d_psi_j;
        d_phi_i;
        d_phi_j;
        
    end
    
    methods
        
        % Constructor
        function obj = Oscillator(name,T,beta,eta,sigma,theta_star,start_time,u_i,u_j,h_psi,I,gamma)
            % Set the name
            obj.name = name;
            
            % Set the constants
            obj.T = T;
            [obj.t1, obj.t2] = derive_time_constants(T); % Derive t1,t2 from T
            obj.beta = beta;
            obj.eta = eta;
            obj.sigma = sigma;
            % couplings will be set once all the oscillator objects
            % have been created and so it is not set in the constructor
            obj.theta_star = theta_star;
            obj.start_time = start_time;
            obj.u_i = u_i;
            obj.u_j = u_j;
            obj.h_psi = h_psi;
            obj.I = I;
            obj.gamma = gamma;
            
            % Initialize the variables and constants
            obj.psi_i = 0.0;
            obj.psi_j = 1.0;
            obj.phi_i = 0.0;
            obj.phi_j = 1.0;
            obj.d_psi_i = 0.0;
            obj.d_psi_j = 0.0;
            obj.d_phi_i = 0.0;
            obj.d_phi_j = 0.0;       
            obj.psi = 0.0;
            obj.theta = 0.0;
        end
        
    end
    
end

