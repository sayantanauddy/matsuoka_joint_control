function div = check_diversity(population, swarm_size, individual_length)

%check_diversity - Function to calculate the diversity of a population. 
% Refer to http://ieeexplore.ieee.org/document/4424896/
%
% Syntax:  div = check_diversity(population, swarm_size, individual_length)
%
% Inputs:
%    population - A list of structures consisting, each structure being an
%    individual solution. The structure contains the following fields: 
%    'position', 'velocity', 'personal_best', 'personal_best_fitness'
%    swarm_size - Number of individuals in the swarm or population.
%    individual_length - Dimension of an individual solution
%
%
% Outputs:
%    div - Diversity of the population
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author: Sayantan Auddy
% email: 
% Website: 
% Dec 2016; Last revision: 11-Dec-2016

%------------- BEGIN CODE --------------   

    % Calculate average values for each dimension of the individuals
    x_average = zeros(1,individual_length);
    for i = 1:swarm_size
        individual = population(i);
        x = individual.position;
        for j = 1:individual_length
            x_average(j) = x_average(j) + x(j);
        end
    end
    x_average = x_average / swarm_size;
    
    % Calculate the diversity
    outer = 0.0;
    for i = 1:swarm_size
        inner = 0.0;
        individual = population(i);
        x = individual.position;
        for j = 1:individual_length
            inner = inner + (x(j) - x_average(j))^2;
        end
        outer = outer + sqrt(inner);
    end
    
    div = outer/swarm_size;
    
end

%------------- END OF CODE --------------
