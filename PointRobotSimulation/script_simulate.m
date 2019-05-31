clear all; clc; close all;

%% R2 simulation
N = 2000; %number of simulations
vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'KL', 'DISAMB'};
%for R2 case
for ii=1:N
    %define global variables
   tic;
   fprintf('R2 sim run %d\n', ii);
   SimulateTraj_R2_With_Blending;
   filename = strcat('R22_DATA/RUN_3_DATAR22_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type','random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   clear all; clc; close all;
   N = 2000; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'KL', 'DISAMB'};
   toc;  
end

%% R3 simulation.

N = 2000;
vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'KL', 'DISAMB'};
for ii=1:N
    tic;
    fprintf('R3 sim run %d\n', ii);
    %define global variables
   SimulateTraj_R3_With_Blending;
   filename = strcat('R33_DATA/RUN_3_DATAR33_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type','random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   
   clear all; clc; close all;
   N = 2000; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'KL', 'DISAMB'};
   toc;
end

%% SE2 SIMULATION
N = 2000;
vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'KL', 'DISAMB'};
for ii=1:N
    %define global variables
   tic;
   fprintf('SE2 sim run %d\n', ii);
   SimulateTraj_SE2_With_Blending;
   filename = strcat('SE2_DATA/RUN_3_DATASE2_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type', 'random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   
   clear all; clc; close all;
   N = 2000; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'KL', 'DISAMB'};
   toc;
end

%% SE3 SIMULATION

N= 2000;
vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'KL', 'DISAMB'};
for ii=1:N
    tic;
    %define global variables
   fprintf('SE3 sim run %d\n', ii);
   SimulateTraj_SE3_With_Blending;
   filename = strcat('SE3_DATA/RUN_3_DATASE3_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg_T', 'xr_T_true', 'intent_type', 'random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   clear all; clc; close all;
   N = 2000; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'KL', 'DISAMB'};
   toc;
end