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
        theta_star; % Reference position of oscillation
        u_i; % Tonic input to neuron i
        u_j; % Tonic input to neuron j
        h_psi; % Torque gain factor
        I; % Moment of inertia
        gamma; % Damping constant
        start_time; % Time after t=0 when oscillator should be switched on
        couplings; % List of Coupling objects (with other oscillators)

        
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
        function obj = Oscillator(name,T,beta,eta,sigma,theta_star,u_i,u_j,h_psi,I,gamma,start_time)
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
            obj.u_i = u_i;
            obj.u_j = u_j;
            obj.h_psi = h_psi;
            obj.I = I;
            obj.gamma = gamma;
            obj.start_time = start_time;
            
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
        
        function print_str = print(obj)
                            
            % Retrieve the couplings
            coupling_str = '';        
            coupling_keys = keys(obj.couplings);
            for i = 1:length(obj.couplings)
                coupling_str = strcat(coupling_str,...
                               obj.name,'->', obj.couplings(coupling_keys{i}).coupled_with, ...
                               ' mu=', num2str(obj.couplings(coupling_keys{i}).mu),...
                               ' nu=', num2str(obj.couplings(coupling_keys{i}).nu),...
                               '\n');
            end
            
            % Retrieve the other object parameters
            print_str = strcat(...
                         '\nOscillator name: ',obj.name, '\n'...
                        ,'Parameters: \n'...
                        ,'T=', num2str(obj.T)...
                        ,' t1=', num2str(obj.t1)...
                        ,' t2=', num2str(obj.t2)...
                        ,' beta=', num2str(obj.beta)...
                        ,' eta=', num2str(obj.eta)...
                        ,' sigma=', num2str(obj.sigma)...
                        ,' theta_star=', num2str(obj.theta_star)...
                        ,' u_i=', num2str(obj.u_i)...
                        ,' u_j=', num2str(obj.u_j)...
                        ,' h_psi=', num2str(obj.h_psi)...
                        ,' I =', num2str(obj.I)...
                        ,' gamma=', num2str(obj.gamma)...
                        ,' start_time=', num2str(obj.start_time), '\n'...
                        ,'Couplings:\n'...
                        ,coupling_str...
                        );
        end
        
    end
    
end

