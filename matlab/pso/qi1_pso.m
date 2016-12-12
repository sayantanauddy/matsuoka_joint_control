
function qi1_pso(position_bounds, swarm_size, max_iters, C1, C2, d_low, d_high, sobol_flag)

%qi1_pso - Function for optimization using ATRE-PSO (a modification of
%atraction repulsion PSO). Refer to http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4344042
%
% Syntax:  qi1_pso(position_bounds, swarm_size, max_iters, C1, C2, d_low, d_high, sobol_flag)
%
% Inputs:
%    position_bounds - A 2D array where each row contains 2 numbers - the
%    lower bound and the upper bound for the respective dimension. The row
%    number corresponds to the dimension number.
%    swarm_size - Number of individuals in the swarm or population.
%    max_iters - Maximum number of iterations to perform
%    C1 - PSO constant
%    C2 - PSO constant
%    d_low - Lower threshold of diversity (used for switching between
%    attraction, repulsion or in-between mode, see paper for details).
%    d_high - Upper threshold for diversity
%    sobol_flag - Flag to determine if initialization is to be done using a
%    sobol sequence. If false, the initialization is done using a uniform
%    distribution
%
%
% Outputs:
%    None
%
% Example: 
%    position_bounds = [0 1;0 1;0 1;0 1;0 1;0 1];
%    qi1_pso(position_bounds, 30, 10000, 2.0, 2.0, 0.000005, 0.25, true)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: hart6sc.m, check_diversity.m
%
% See also: 

% Author: Sayantan Auddy
% email: 
% Website: 
% Dec 2016; Last revision: 11-Dec-2016

%------------- BEGIN CODE --------------        

    individual_length = length(position_bounds);
    
    % Initialize the swarm
    population = [];
    
    g_best_position = zeros(1, individual_length);
    g_best_fitness = Inf(1);
    
    % Whether to use the Sobol sequence or not is decided by the input
    % paramater sobol_flag
    % Generate a Sobol point set (dimensions=individual_length), skip the 
    % first 1000 values, and then retain every 101st point: 
    p = sobolset(individual_length,'Skip',1e3,'Leap',1e2);
    % Use scramble to apply a random linear scramble combined with a random 
    % digital shift
    p = scramble(p,'MatousekAffineOwen');
    % Use net to generate 'swarm_size' points
    % sobol_points contains swarm_size number of rows, each of dimension
    % equal to individual_length. Each number is a random number between 0
    % and 1.
    sobol_points = net(p,swarm_size);
    
    for i = 1:swarm_size
        
        % Initialize an individual
        x = zeros(1, individual_length);
        v = zeros(1, individual_length);
        p_best_position = zeros(1, individual_length);
        p_best_fitness = Inf(1);
        
        for j = 1:individual_length
            % Initialize the position as a random number between the upper
            % and lower bounds if sobol_flag=false else use the sobol
            % sequence
            if sobol_flag
                x(j) = position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*sobol_points(i,j);
            else
                x(j) = position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1);
            end
            
            % Initialize the velocity according to SPSO-07
            v(j) = 0.5*((position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)) - (position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)));
        end
        
        individual = struct('position', x, 'velocity', v, 'personal_best', p_best_position, 'personal_best_fitness', p_best_fitness);
        population = [population individual];
    end

    % Main PSO loop
    for iteration = 1:max_iters
        
        % Set the inertial weight
        w = 0.9 - (iteration*(0.9 - 0.4)/max_iters);
        
        % Check the diversity of the population
        div = check_diversity(population, swarm_size, individual_length);
        
        % Decide between the attraction, in-between or repulsion mode
        % Mode for basic PSO is attraction
        mode = 1; % for attraction 
        if div>d_high
            mode = 1;
        elseif (div>=d_low) & (div<=d_high)
            mode = 2; % for in-between
        elseif div<d_low
            mode = 3; % for repulsion
        end
        
        for index = 1:swarm_size
            individual = population(index);
            % Unpack the individual
            x = individual.position;
            v = individual.velocity;
            p_best_position = individual.personal_best;
            p_best_fitness = individual.personal_best_fitness;
            
            % Calculate the fitness of the individual
            indiv_fitness = hart6sc(x);
            
            % Check pbest
            if(indiv_fitness<p_best_fitness)
                p_best_position = x;
                p_best_fitness = indiv_fitness;
                individual = struct('position', x, 'velocity', v, 'personal_best', p_best_position, 'personal_best_fitness', p_best_fitness);
                population(index) = individual;
            end
            
            % Check gbest
            if(indiv_fitness<g_best_fitness)
                g_best_fitness = indiv_fitness;
                g_best_position = x;
                %disp(g_best_fitness);
            end
            
        end
            
            
        % Once pbest and gbest have been determined, x and v need to be updated
        for index = 1:swarm_size
            individual = population(index);
            
            % Unpack the individual
            x = individual.position;
            v = individual.velocity;
            p_best_position = individual.personal_best;
            p_best_fitness = individual.personal_best_fitness;
            
            % Generate 2 random numbers in the range 0.0 to 1.0
            rand1 = 0.0 + (1.0 - 0.0)*rand(1,1);
            rand2 = 0.0 + (1.0 - 0.0)*rand(1,1);
            
            % Calculate new velocity and position
            for pos_index = 1:individual_length
                old_vel = v(pos_index);
                
                % Update the velocity based on the mode (attraction,
                % in-between or repulsion)
                switch mode
                    case 1
                        new_vel = w*old_vel + C1*rand1*(p_best_position(pos_index) - x(pos_index)) +  C2*rand2*(g_best_position(pos_index) - x(pos_index));
                    case 2
                        new_vel = w*old_vel + C1*rand1*(p_best_position(pos_index) - x(pos_index)) -  C2*rand2*(g_best_position(pos_index) - x(pos_index));
                    case 3
                        new_vel = w*old_vel - C1*rand1*(p_best_position(pos_index) - x(pos_index)) -  C2*rand2*(g_best_position(pos_index) - x(pos_index));
                end
                
                v(pos_index) = new_vel;
                
                % Calculate new positions
                new_pos = x(pos_index) + new_vel;
                
                %  Check the position bounds
                if new_pos<position_bounds(pos_index,1)
                    new_pos = position_bounds(pos_index,1);
                end
                
                if new_pos>position_bounds(pos_index,2)
                    new_pos = position_bounds(pos_index,2);
                end
                    
                x(pos_index) = new_pos;
            end
            
            individual = struct('position', x, 'velocity', v, 'personal_best', p_best_position, 'personal_best_fitness', p_best_fitness);
            population(index) = individual;
            
        end 
    end  
    
     disp( sprintf( 'Best fitness: %f at location: %s', g_best_fitness, mat2str(g_best_position) ) )
end

%------------- END OF CODE --------------
