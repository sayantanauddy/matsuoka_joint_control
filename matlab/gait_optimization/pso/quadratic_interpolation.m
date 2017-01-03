function  [population]=quadratic_interpolation(population, swarm_size, individual_length, position_bounds)

%quadratic_interpolation - Function for creating new individuals by 
%Quadratic Interpolation using the best individual and two other randomly 
%chosen individuals. The new individual is added to the population which is 
%then returned by the function.
%
% Syntax:  quadratic_interpolation(population, swarm_size, individual_length, position_bounds)
%
% Inputs:

%    population - A population (swarm) of individuals. Each individual is a
%    structure consisting of the following fields: 'position', 'velocity', 
%    'personal_best', 'personal_best_fitness'
%    swarm_size - Number of individuals in the swarm or population.
%    individual_length - Number of elements in an individual's position
%    vector
%    position_bounds - A 2D array where each row contains 2 numbers - the
%    lower bound and the upper bound for the respective dimension. The row
%    number corresponds to the dimension number.
%
%
% Outputs:
%    population - A new population, with the newly generated individual
%    replacing the worst individual
%
% Example: 
%    position_bounds = [0 1;0 1;0 1;0 1;0 1;0 1];
%    qi1_pso(position_bounds, 30, 10000, 2.0, 2.0, 0.000005, true)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: None
%
% See also: 

% Author: Sayantan Auddy
% email: 
% Website: 
% Dec 2016; Last revision: 12-Dec-2016

%------------- BEGIN CODE --------------  

    % Determine the best and the worst individual in the population
    best_fitness = Inf(1);
    worst_fitness = -Inf(1);
    best_index = 0;
    worst_index = 0;
    for i=1:swarm_size
        individual = population(i);
        individual_fitness = individual.personal_best_fitness;
        if individual_fitness<best_fitness
            best_fitness = individual_fitness;
            best_individual = individual;
            best_index = i;
        end
        if individual_fitness>worst_fitness
            worst_fitness = individual_fitness;
            worst_index = i;
        end
    end
    
    % Randomly pick 2 other individuals (apart from the best)
    index1 = randi([1 swarm_size]);
    index2 = randi([1 swarm_size]);
    while index1 == best_index || index2== best_index
        index1 = randi([1 swarm_size]);
        index2 = randi([1 swarm_size]);
    end
    rand_individual1 = population(index1);
    rand_individual2 = population(index2);
    
    % Generate a new individual using the p_best locations of the best and
    % the other 2 random individuals
    a = best_individual.personal_best;
    fa = best_fitness;
    b = rand_individual1.personal_best;
    fb = rand_individual1.personal_best_fitness;
    c = rand_individual2.personal_best;
    fc = rand_individual2.personal_best_fitness;
    
    new_individual = zeros(1, individual_length);
    
    for i=1:individual_length
        new_individual(i) = 0.5*((b(i)^2 - c(i)^2)*fa + (c(i)^2 - a(i)^2)*fb + (a(i)^2 - b(i)^2)*fc) / ((b(i) - c(i))*fa + (c(i) - a(i))*fb + (a(i) - b(i))*fc);
    end
    
    % Compute the fitness of this new individual
    new_individual_fitness = hart6sc(new_individual);
    
    % Replace the worst individual by the new individual 
    % (slight modification to QIPSO-1, in which the new individual is added 
    % only if its fitness is better than the fitness of the worst individual)
    new_vel = zeros(1, individual_length);
    for j = 1:individual_length
        % Initialize the velocity according to SPSO-07
        new_vel(j) = 0.5*((position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)) - (position_bounds(j,1) + (position_bounds(j,2) - position_bounds(j,1))*rand(1,1)));
    end
    population(worst_index) = struct('position', new_individual, 'velocity', new_vel, 'personal_best', new_individual, 'personal_best_fitness', new_individual_fitness);

end


%------------- END OF CODE --------------