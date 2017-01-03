
function basic_pso(position_bounds, swarm_size, max_iters, C1, C2)

%basic_pso - Function for optimization using a basic version of PSO
%
% Syntax:  basic_pso(position_bounds, swarm_size, max_iters, C1, C2)
%
% Inputs:
%    position_bounds - A 2D array where each row contains 2 numbers - the
%    lower bound and the upper bound for the respective dimension. The row
%    number corresponds to the dimension number.
%    swarm_size - Number of individuals in the swarm or population.
%    max_iters - Maximum number of iterations to perform
%    C1 - PSO constant
%    C2 - PSO constant
%
% Outputs:
%    None
%
% Example: 
%    position_bounds = [0 1;0 1;0 1;0 1;0 1;0 1];
%    basic_pso(position_bounds, 30, 10000, 2.0, 2.0)
%
% Other m-files required: hart6sc.m
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author: Sayantan Auddy
% email: 
% Website: 
% Dec 2016; Last revision: 11-Dec-2016

%------------- BEGIN CODE --------------      

    log_write('Started the PSO algorithm: basic_pso');
    log_write('PSO parameters');
    log_write(['swarm_size=',num2str(swarm_size), ' max_iters=',num2str(max_iters), ' C1=',num2str(C1), ' C2=',num2str(C2)]);

    individual_length = length(position_bounds);
    
    % Initialize the swarm
    population = [];
    
    g_best_position = zeros(1, individual_length);
    g_best_fitness = Inf(1);
    
    for i = 1:swarm_size
        
        % Initialize an individual
        x = zeros(1, individual_length);
        v = zeros(1, individual_length);
        p_best_position = zeros(1, individual_length);
        p_best_fitness = Inf(1);
        
        for j = 1:individual_length
            % Initialize the position as a random number between the upper and lower bounds
            x(j) = position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1);
            
            % Initialize the velocity according to SPSO-07
            v(j) = 0.5*((position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)) - (position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)));
        end
        
        individual = struct('position', x, 'velocity', v, 'personal_best', p_best_position, 'personal_best_fitness', p_best_fitness);
        population = [population individual];
    end
    
    log_write('Initialized population');

    log_write('Starting PSO loop');
    
    % Main PSO loop
    for iteration = 1:max_iters
        
        log_write(['Started iteration # ', num2str(iteration)]);
        
        % Set the inertial weight
        w = 0.9 - (iteration*(0.9 - 0.4)/max_iters);
        log_write(['inertial weight=', num2str(w)]);
        
        for index = 1:swarm_size
            individual = population(index);
            % Unpack the individual
            x = individual.position;
            v = individual.velocity;
            p_best_position = individual.personal_best;
            p_best_fitness = individual.personal_best_fitness;
            
            
            % Calculate the fitness of the individual
            indiv_fitness = eval_gait_fitness(x);
            
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
                log_write(['New GBest: ', num2str(g_best_fitness)]);
                % Extract the network and log the result
                [ gb_l1, gb_l2, gb_l3, gb_l4, gb_l5, gb_r1, gb_r2, gb_r3, gb_r4, gb_r5, gb_coupling_start_time, gb_joint_start_time ] = extract_network( x );
                log_write(sprintf(print_network(gb_l1, gb_l2, gb_l3, gb_l4, gb_l5, gb_r1, gb_r2, gb_r3, gb_r4, gb_r5, gb_coupling_start_time, gb_joint_start_time)));
            end
            
        end
            
            
        % Once pbest and gbest have been determined, x and v need to be updated
        log_write('Updating position and velocity');
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
                new_vel = w*old_vel + C1*rand1*(p_best_position(pos_index) - x(pos_index)) +  C2*rand2*(g_best_position(pos_index) - x(pos_index));
                
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
    % Extract the network and log the result
    log_write(sprintf('Best fitness: %f at location: %s',g_best_fitness, mat2str(g_best_position)));
    [ gb_l1, gb_l2, gb_l3, gb_l4, gb_l5, gb_r1, gb_r2, gb_r3, gb_r4, gb_r5, gb_coupling_start_time, gb_joint_start_time ] = extract_network( g_best_position );
    log_write(sprintf(print_network(gb_l1, gb_l2, gb_l3, gb_l4, gb_l5, gb_r1, gb_r2, gb_r3, gb_r4, gb_r5, gb_coupling_start_time, gb_joint_start_time)));
    
    log_write('END');
    
end

%------------- END OF CODE --------------
