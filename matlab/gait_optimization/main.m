
% Add the subfolders to the path
addpath('pso');
addpath('config');
addpath('logger');
addpath('log_files');
addpath('classes');
addpath('ode');
addpath('matsuoka');
addpath('param_files');

% Create a log file
global log_file;
log_file = strcat('log_files/matsuoka_pso_gait_', datestr(now,'yyyymmdd_HHMMSS'), '.log');

% Parameters are stored in this XML file
global xml_file;
xml_file = 'param_files/nao_parameters.xml';

% Set the time vector
global time_vector;
time_vector = 0:0.05:30;

log_write('BEGIN');

% Create the position_bounds array for the pso algorithm by reading the max 
% and min param values from the config file
log_write('Creating position bounds from config file');
[T_min, T_max] = get_params( 'T' );
[beta_min, beta_max] = get_params( 'beta' );
[eta_min, eta_max] = get_params( 'eta' );
[sigma_min, sigma_max] = get_params( 'sigma' );
[theta_star_min, theta_star_max] = get_params( 'theta_star' );
[u_min, u_max] = get_params( 'u' );
[h_psi_min, h_psi_max] = get_params( 'h_psi' );
[I_min, I_max] = get_params( 'I' );
[gamma_min, gamma_max] = get_params( 'gamma' );
[mu_min, mu_max] = get_params( 'mu' );
[nu_min, nu_max] = get_params( 'nu' );
[oscillator_start_time_min, oscillator_start_time_max] = get_params( 'oscillator_start_time' );
[coupling_start_time_min, coupling_start_time_max] = get_params( 'coupling_start_time' );
[joint_start_time_min, joint_start_time_max] = get_params( 'joint_start_time' );

% The first 45 elements are 5 sets of oscillator parameters for each
% oscillator of one leg. The oscillators of the other leg are symmetric.
% Hence instead of 10 sets, only 5 are required
% (T,beta,eta,sigma,theta_star,u,h_psi,I,gamma) 
oscillator_bounds = [T_min T_max;
                     beta_min beta_max;
                     eta_min, eta_max;
                     sigma_min, sigma_max;
                     theta_star_min, theta_star_max;
                     u_min, u_max;
                     h_psi_min, h_psi_max;
                     I_min, I_max;
                     gamma_min, gamma_max];

oscillator_bounds = [oscillator_bounds; oscillator_bounds; oscillator_bounds;
                     oscillator_bounds; oscillator_bounds];
               
% The next 12 elements are 6 pairs of (mu, nu) for the coupling configuration               
coupling_bounds =  [mu_min mu_max;
                    nu_min nu_max];

coupling_bounds = [coupling_bounds; coupling_bounds; coupling_bounds;
                   coupling_bounds; coupling_bounds; coupling_bounds];
                   
% The next 6 elements are numbers which will be used to derive the             
% oscillator_start_tims which denotes when an oscillator should start (for initial phase difference)
oscillator_start_bounds = [oscillator_start_time_min oscillator_start_time_max];

oscillator_start_bounds = [oscillator_start_bounds; oscillator_start_bounds;
                           oscillator_start_bounds; oscillator_start_bounds;
                           oscillator_start_bounds; oscillator_start_bounds];

% The last 2 elements are:                       
% coupling_start_time: When the coupling between the oscillators should be turned on
% joint_start_time: When the robot joints should be activated
 
time_bounds = [coupling_start_time_min coupling_start_time_max;
               joint_start_time_min joint_start_time_max];

% position_bounds is a 65x2 array, where col 1->min val and col 2->max val
position_bounds = [oscillator_bounds; coupling_bounds; oscillator_start_bounds; time_bounds];
    
% Call the PSO algorithm
log_write('Executing the PSO algorithm');
basic_pso(position_bounds, 30, 1000, 2.0, 2.0)
%atre_pso(position_bounds, 30, 1000, 2.0, 2.0, 0.000005, 0.25, true)
%qi1_pso(position_bounds, 30, 1000, 2.0, 2.0, 0.000005, 0.25, true)


