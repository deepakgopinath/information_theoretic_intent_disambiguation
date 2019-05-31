% clear all; close all; clc;
%% SPAWN GOAL CONFIGURATION AND ROBOT POSITION. RANDOMLY IN THE WORKSPACE. 

%Define workspace limits. All in metres. 
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
th_range = [0, 2*pi]; %counter clockwise rotation with 0 as x-axis aligned with left-right directions of the plane. 

global num_modes cm ng nd xg xr sig delta_t conf_thresh conf_max alpha_max sparsity_factor num_samples amp_sparsity_factor kappa projection_time;
max_ng = 6;
ng = datasample(3:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 3; %num of dimensions. by definition SE2
cm_options = {{1,2,3}, {[1,2], 3}};
max_nm = length(cm_options);
cm = cm_options{datasample(1:max_nm, 1)};
num_modes = length(cm);
init_mode_index = datasample(1:num_modes, 1);

xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(th_range) + th_range(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(th_range) + th_range(1)];
xr_true = xr;

%human parameters
sparsity_factor = rand/8;
amp_sparsity_factor = rand/8; % how often the amplitude wiull be less that maximum. 
kappa = 20.0; % concentration paarameter for vonMisesFisher distribution
fprintf('The sparsity and amp factor are %f, %f\n', sparsity_factor, amp_sparsity_factor);

%disamb parameters.
sig = 0.01; %For Fisher information
%% Projection paramaters
projection_time = 4;
delta_t = 0.1; %For compute projections. 
num_samples = 5;
%% simulation params
mode_comp_timesteps = 10; %time step gap between optimal mode computation. delta_t*mode_comp_timesteps is the time in seconds
exit_threshold = 0.02;
total_time_steps = 120; %with delta_t of 0.1, this amounts to 10 seconds. We will assume that "mode switches" don't take time. 

%% arbitration function parameters
conf_thresh = (1.05/ng);
conf_max = (1.1/ng);
alpha_max = 0.7;

%% plot positions and body rotation axis. 
% figure;
% scatter(xg(1,1:ng), xg(2,1:ng), 230, 'k', 'filled'); grid on; hold on;
% scatter(xr(1), xr(2), 140, 'r', 'filled');
% % for i=1:ng %vectors connecting robot and goals.
% %     quiver(xr(1), xr(2), xg(1,i) - xr(1), xg(2,i) - xr(2), 'LineWidth', 1.5, 'LineStyle', '-.');
% % end
% offset = [-0.1, 0.1];
% line(xrange+offset, [0,0], 'Color', 'r'); %draw x and y axes.
% line([0,0], yrange+offset, 'Color', 'g');
% axis([xrange + offset, yrange + offset]);
% axis square;
% 
% % draw orientations:
% l_axis = 0.05; % length of axis 
% line([xr(1), xr(1) + l_axis*cos(xr(3))], [xr(2), xr(2) + l_axis*sin(xr(3))], 'Color', 'r', 'LineWidth', 2);
% line([xr(1), xr(1) - l_axis*sin(xr(3))], [xr(2), xr(2) + l_axis*cos(xr(3))], 'Color', 'g', 'LineWidth', 2);
% for i=1:ng
%     line([xg(1, i), xg(1, i) + l_axis*cos(xg(3, i))], [xg(2, i), xg(2, i) + l_axis*sin(xg(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([xg(1, i), xg(1, i) - l_axis*sin(xg(3, i))], [xg(2, i), xg(2, i) + l_axis*cos(xg(3, i))], 'Color', 'g', 'LineWidth', 2);
% end
%% Generate the random goal towards which the simulated human would move. 
random_goal_index = randsample(ng, 1);
random_goal = xg(:, random_goal_index);
% scatter(random_goal(1), random_goal(2), 230, 'm', 'filled'); grid on; hold on;

%% Sample random intent inference type
intent_types = {'dft', 'conf', 'bayes'};
intent_type = intent_types{datasample(1:length(intent_types), 1)}; % or conf or bayes
% intent_type = 'dft';
%% BASELINE
pgs_POT = zeros(ng, total_time_steps);
optimal_modes_POT = zeros(total_time_steps-1, 1);
alpha_POT = zeros(total_time_steps-1, 1);
uh_POT = zeros(nd, total_time_steps-1);
ur_POT = zeros(nd, total_time_steps-1);
blend_vel_POT = zeros(nd, total_time_steps-1);
curr_goal_POT = zeros(total_time_steps-1, 1);
traj_POT = zeros(nd, total_time_steps);
traj_POT(:, 1) = xr;
pgs_POT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection
current_optimal_mode_POT_index = init_mode_index;
current_optimal_mode_POT = cm{current_optimal_mode_POT_index};

for i=1:total_time_steps-1
    if norm(traj_POT(1:2, i) - random_goal(1:2))/norm(traj_POT(1:2, 1)- random_goal(1:2)) < exit_threshold && (traj_POT(3, i) - random_goal(3)) < 0.05 %position is within exit threshold percent and angle is less than 0.05 radians exit
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_POT = datasample(find(pgs_POT(:, i) == max(pgs_POT(:, i))), 1);
    curr_goal_POT(i) = curr_goal_index_POT;
    %compute the optimal mode. store it away. 
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_POT_index = compute_optimal_mode_POT_SE2(xg(:, curr_goal_index_POT), xr_true); 
        if length(current_optimal_mode_POT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_POT = cm{current_optimal_mode_POT_index(1)}; %pick the first one. 
            current_optimal_mode_POT_index = current_optimal_mode_POT_index(1); %update the index. 
        else
            if current_optimal_mode_POT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_POT = cm{current_optimal_mode_POT_index}; 
            end
        end
    end
    %gotta determine uh. Assumes human is executes the "straightest
    %possible motion in the current mode towards the specified goal". with
    %some noise
    uh = generate_full_uh(random_goal, xr); 
    zero_dim = setdiff(1:nd,current_optimal_mode_POT);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     uh = 0.2*(uh./(abs(uh) + realmin));
    if norm(uh(1:2)) > 0.2
        uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
    end
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    ur = generate_autonomy(curr_goal_index_POT); %autonomy command in full 2D space toward what it thinks is the current goal
    alpha_POT(i) = alpha_from_confidence(pgs_POT(curr_goal_index_POT, i)); %linear belnding param
    blend_vel = (1-alpha_POT(i))*uh + alpha_POT(i)*ur; %blended vel
    uh_POT(:, i) = uh; ur_POT(:, i) = ur; blend_vel_POT(:, i) = blend_vel;
    
    %forward propragate belief based on one of the three intent inference
    %mechanisms. 
    if strcmp(intent_type, 'dft')
        pgs_POT(:, i+1) = compute_p_of_g_dft_SE2(uh, xr, pgs_POT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_POT(:, i+1) = compute_bayes_SE2(uh, xr, pgs_POT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_POT(:, i+1) = compute_conf_SE2(uh, xr);
    end
    
    %simulate kinematics. 
    xr = sim_kinematics_R3(xr, blend_vel); %forward simulate kinematics with blending. 
    
    %update trajectories. 
    traj_POT(:, i+1) = xr;
    
    optimal_modes_POT(i) = current_optimal_mode_POT_index;
end
%% ENTROPY BASED DISAMBIGUATION
fprintf("SIMULATION USING ENTROPY INFORMATION \n");
xr = xr_true;
pgs_ENT = zeros(ng, total_time_steps);
optimal_modes_ENT = zeros(total_time_steps-1, 1);
alpha_ENT = zeros(total_time_steps-1, 1);
uh_ENT = zeros(nd, total_time_steps-1);
ur_ENT = zeros(nd, total_time_steps-1);
blend_vel_ENT = zeros(nd, total_time_steps-1);
curr_goal_ENT = zeros(total_time_steps-1, 1);
traj_ENT = zeros(nd, total_time_steps);
traj_ENT(:, 1) = xr; %(x,y,theta)
pgs_ENT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_ENT_index = init_mode_index;
current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index};

for i=1:total_time_steps-1
    if norm(traj_ENT(1:2, i) - random_goal(1:2))/norm(traj_ENT(1:2, 1)- random_goal(1:2)) < exit_threshold && (traj_ENT(3, i) - random_goal(3)) < 0.05
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_ENT = datasample(find(pgs_ENT(:, i) == max(pgs_ENT(:, i))), 1);
    curr_goal_ENT(i) = curr_goal_index_ENT;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_ENT_index = compute_optimal_mode_ENT_SE2_human_model(intent_type, xr, pgs_ENT(:, i)); 
        if length(current_optimal_mode_ENT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index(1)}; %pick the first one. 
            current_optimal_mode_ENT_index = current_optimal_mode_ENT_index(1); %update the index. 
        else
            if current_optimal_mode_ENT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index}; 
            end
        end
    end
    uh = generate_full_uh(random_goal, xr); %only 
    zero_dim = setdiff(1:nd,current_optimal_mode_ENT);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     uh = 0.2*(uh./(abs(uh) + realmin));
    if norm(uh(1:2)) > 0.2
        uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
    end
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    ur = generate_autonomy(curr_goal_index_ENT);
    alpha_ENT(i) = alpha_from_confidence(pgs_ENT(curr_goal_index_ENT, i)); %linear belnding param
    blend_vel = (1-alpha_ENT(i))*uh + alpha_ENT(i)*ur; %blended vel
    uh_ENT(:, i) = uh; ur_ENT(:, i) = ur; blend_vel_ENT(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_ENT(:, i+1) = compute_p_of_g_dft_SE2(uh, xr, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_ENT(:, i+1) = compute_bayes_SE2(uh, xr, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_ENT(:, i+1) = compute_conf_SE2(uh, xr);
    end
    xr = sim_kinematics_SE2(xr, blend_vel);
    traj_ENT(:, i+1) = xr;
    optimal_modes_ENT(i) = current_optimal_mode_ENT_index; 
end

%% USING KL DIVERGENCE
fprintf("SIMULATION USING KL DIVERGENCE INFORMATION \n");
xr = xr_true; %reset the robot position to starting position
pgs_KL = zeros(ng, total_time_steps);
optimal_modes_KL = zeros(total_time_steps-1, 1);
alpha_KL = zeros(total_time_steps-1, 1);
uh_KL = zeros(nd, total_time_steps-1);
ur_KL = zeros(nd, total_time_steps-1);
blend_vel_KL = zeros(nd, total_time_steps-1);
curr_goal_KL = zeros(total_time_steps-1, 1);
traj_KL = zeros(nd, total_time_steps);
traj_KL(:, 1) = xr;
pgs_KL(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection
current_optimal_mode_KL_index = init_mode_index;
current_optimal_mode_KL = cm{current_optimal_mode_KL_index};

for i=1:total_time_steps-1
    if norm(traj_KL(1:2, i) - random_goal(1:2))/norm(traj_KL(1:2, 1)- random_goal(1:2)) < exit_threshold && (traj_KL(3, i) - random_goal(3)) < 0.05
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    %compute the optimal mode. store it away. 
    curr_goal_index_KL = datasample(find(pgs_KL(:, i) == max(pgs_KL(:, i))), 1);
    curr_goal_KL(i) = curr_goal_index_KL;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_KL_index = compute_optimal_mode_KL_SE2_human_model(intent_type, xr, pgs_KL(:, i)); 
        if length(current_optimal_mode_KL_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_KL = cm{current_optimal_mode_KL_index(1)}; %pick the first one. 
            current_optimal_mode_KL_index = current_optimal_mode_KL_index(1); %update the index. 
        else
            if current_optimal_mode_KL_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_KL = cm{current_optimal_mode_KL_index}; 
            end
        end
    end
    %gotta determine uh. Assumes human is executes the "straightest
    %possible motion in the current mode towards the specified goal". 
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_KL);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     uh = 0.2*(uh./(abs(uh) + realmin));
    if norm(uh(1:2)) > 0.2
        uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
    end
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    ur = generate_autonomy(curr_goal_index_KL); %autonomy command in full 2D space
    alpha_KL(i) = alpha_from_confidence(pgs_KL(curr_goal_index_KL, i)); %linear belnding param
    blend_vel = (1-alpha_KL(i))*uh + alpha_KL(i)*ur; %blended vel
    uh_KL(:, i) = uh; ur_KL(:, i) = ur; blend_vel_KL(:, i) = blend_vel;

    if strcmp(intent_type, 'dft')
        pgs_KL(:, i+1) = compute_p_of_g_dft_SE2(uh, xr, pgs_KL(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_KL(:, i+1) = compute_bayes_SE2(uh, xr, pgs_KL(:, i));
%         pgs_KL(:, i+1) = compute_bayes_n_R2(uh_KL(:, max(1, i-hist_length+1):i), xr, pgs_KL(:,i));
    elseif strcmp(intent_type, 'conf')
        pgs_KL(:, i+1) = compute_conf_SE2(uh, xr);
    end
    
    xr = sim_kinematics_SE2(xr, blend_vel); %forward simulate kinematics with blending. 
    traj_KL(:, i+1) = xr;
    optimal_modes_KL(i) = current_optimal_mode_KL_index;
end


% %% FISHER INFO BAsed
% fprintf("SIMULATION USING FISHER INFORMATION \n");
% xr = xr_true;
% pgs_FI = zeros(ng, total_time_steps);
% optimal_modes_FI = zeros(total_time_steps-1, 1);
% alpha_FI = zeros(total_time_steps-1, 1);
% uh_FI = zeros(nd, total_time_steps-1);
% ur_FI = zeros(nd, total_time_steps-1);
% blend_vel_FI = zeros(nd, total_time_steps-1);
% curr_goal_FI = zeros(total_time_steps-1, 1);
% traj_FI = zeros(nd, total_time_steps);
% 
% traj_FI(:, 1) = xr; %(x,y,theta)
% pgs_FI(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
% %internally projection
% 
% current_optimal_mode_FI_index = 1;
% current_optimal_mode_FI = cm{current_optimal_mode_FI_index};
% 
% for i=1:total_time_steps-1
%     curr_goal_index_FI = datasample(find(pgs_FI(:, i) == max(pgs_FI(:, i))), 1);
%     curr_goal_FI(i) =  curr_goal_index_FI;
%     if mod(i-1, mode_comp_timesteps) == 0
%         current_optimal_mode_FI_index = compute_optimal_mode_FI_SE2(intent_type, xr, pgs_FI(:, i)); 
%         if length(current_optimal_mode_FI_index) > 1 %when there are equivalent modes. 
%             current_optimal_mode_FI = cm{current_optimal_mode_FI_index(1)}; %pick the first one. 
%             current_optimal_mode_FI_index = current_optimal_mode_FI_index(1); %update the index. 
%         else
%             if current_optimal_mode_FI_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
%                 current_optimal_mode_FI = cm{current_optimal_mode_FI_index}; 
%             end
%         end
%     end
%     uh = generate_full_uh(random_goal, xr);
%     zero_dim = setdiff(1:nd,current_optimal_mode_FI);
%     for jj=1:length(zero_dim)
%         uh(zero_dim(jj)) = 0;
%     end
%     uh = 0.2*(uh./(abs(uh) + realmin));
%     ur = generate_autonomy(curr_goal_index_FI);
%     alpha_FI(i) = alpha_from_confidence(pgs_FI(curr_goal_index_FI, i)); %linear belnding param
%     blend_vel = (1-alpha_FI(i))*uh + alpha_FI(i)*ur; %blended vel
%     uh_FI(:, i) = uh; ur_FI(:, i) = ur; blend_vel_FI(:, i) = blend_vel;
%     if strcmp(intent_type, 'dft')
%         pgs_FI(:, i+1) = compute_p_of_g_dft_SE2(uh, xr, pgs_FI(:, i));
%     elseif strcmp(intent_type, 'bayes')
%         pgs_FI(:, i+1) = compute_bayes_SE2(uh, xr, pgs_FI(:, i));
%     elseif strcmp(intent_type, 'conf')
%         pgs_FI(:, i+1) = compute_conf_SE2(uh, xr);
%     end
%     xr = sim_kinematics_SE2(xr, blend_vel);
%     traj_FI(:, i+1) = xr;
%     optimal_modes_FI(i) = current_optimal_mode_FI_index; 
% end
%%
%% DISAMB METRIC BASED
fprintf("SIMULATION USING DISAMB METRIC \n");
xr = xr_true;
pgs_DISAMB = zeros(ng, total_time_steps);
optimal_modes_DISAMB = zeros(total_time_steps-1, 1);
alpha_DISAMB = zeros(total_time_steps-1, 1);
uh_DISAMB = zeros(nd, total_time_steps-1);
ur_DISAMB = zeros(nd, total_time_steps-1);
blend_vel_DISAMB = zeros(nd, total_time_steps-1);
curr_goal_DISAMB = zeros(total_time_steps-1, 1);
traj_DISAMB = zeros(nd, total_time_steps);
traj_DISAMB(:, 1) = xr; %(x,y,theta)
pgs_DISAMB(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_DISAMB_index = init_mode_index;
current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index};

for i=1:total_time_steps-1
    if norm(traj_DISAMB(1:2, i) - random_goal(1:2))/norm(traj_DISAMB(1:2, 1)- random_goal(1:2)) < exit_threshold && (traj_DISAMB(3, i) - random_goal(3)) < 0.05
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_DISAMB = datasample(find(pgs_DISAMB(:, i) == max(pgs_DISAMB(:, i))), 1);
    curr_goal_DISAMB(i) = curr_goal_index_DISAMB;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_DISAMB_index = compute_optimal_mode_DISAMB_SE2(intent_type, xr, pgs_DISAMB(:, i)); 
        if length(current_optimal_mode_DISAMB_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index(1)}; %pick the first one. 
            current_optimal_mode_DISAMB_index = current_optimal_mode_DISAMB_index(1); %update the index. 
        else
            if current_optimal_mode_DISAMB_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index}; 
            end
        end
    end
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_DISAMB);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
%     uh = 0.2*(uh./(abs(uh) + realmin));
    if norm(uh(1:2)) > 0.2
        uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
    end
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    ur = generate_autonomy(curr_goal_index_DISAMB);
    alpha_DISAMB(i) = alpha_from_confidence(pgs_DISAMB(curr_goal_index_DISAMB, i)); %linear belnding param
    blend_vel = (1-alpha_DISAMB(i))*uh + alpha_DISAMB(i)*ur; %blended vel
    uh_DISAMB(:, i) = uh; ur_DISAMB(:, i) = ur; blend_vel_DISAMB(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_DISAMB(:, i+1) = compute_p_of_g_dft_SE2(uh, xr, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_DISAMB(:, i+1) = compute_bayes_SE2(uh, xr, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_DISAMB(:, i+1) = compute_conf_SE2(uh, xr);
    end
    xr = sim_kinematics_SE2(xr, blend_vel);
    traj_DISAMB(:, i+1) = xr;
    optimal_modes_DISAMB(i) = current_optimal_mode_DISAMB_index; 
end

%%
% hold on; 
% scatter(traj_POT(1, :)', traj_POT(2, :)', 'k', 'filled');
% scatter(traj_ENT(1, :)', traj_ENT(2, :)', 'r', 'filled');
% scatter(traj_KL(1, :)', traj_KL(2, :)', 'b', 'filled');
% scatter(traj_DISAMB(1, :)', traj_DISAMB(2, :)', 'g', 'filled');
% skip_step = 10;
% for i=1:skip_step:size(traj_POT, 2)
%     line([traj_POT(1, i), traj_POT(1, i) + l_axis*cos(traj_POT(3, i))], [traj_POT(2, i), traj_POT(2, i) + l_axis*sin(traj_POT(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([traj_POT(1, i), traj_POT(1, i) - l_axis*sin(traj_POT(3, i))], [traj_POT(2, i), traj_POT(2, i) + l_axis*cos(traj_POT(3, i))], 'Color', 'g', 'LineWidth', 2);
% end
% 
% for i=1:skip_step:size(traj_ENT, 2)
%     line([traj_ENT(1, i), traj_ENT(1, i) + l_axis*cos(traj_ENT(3, i))], [traj_ENT(2, i), traj_ENT(2, i) + l_axis*sin(traj_ENT(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([traj_ENT(1, i), traj_ENT(1, i) - l_axis*sin(traj_ENT(3, i))], [traj_ENT(2, i), traj_ENT(2, i) + l_axis*cos(traj_ENT(3, i))], 'Color', 'g', 'LineWidth', 2);
% end
% 
% for i=1:skip_step:size(traj_KL, 2)
%     line([traj_KL(1, i), traj_KL(1, i) + l_axis*cos(traj_KL(3, i))], [traj_KL(2, i), traj_KL(2, i) + l_axis*sin(traj_KL(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([traj_KL(1, i), traj_KL(1, i) - l_axis*sin(traj_KL(3, i))], [traj_KL(2, i), traj_KL(2, i) + l_axis*cos(traj_KL(3, i))], 'Color', 'g', 'LineWidth', 2);
% end
% 
% for i=1:skip_step:size(traj_FI, 2)
%     line([traj_FI(1, i), traj_FI(1, i) + l_axis*cos(traj_FI(3, i))], [traj_FI(2, i), traj_FI(2, i) + l_axis*sin(traj_FI(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([traj_FI(1, i), traj_FI(1, i) - l_axis*sin(traj_FI(3, i))], [traj_FI(2, i), traj_FI(2, i) + l_axis*cos(traj_FI(3, i))], 'Color', 'g', 'LineWidth', 2);
% end

% for i=1:skip_step:size(traj_DISAMB, 2)
%     line([traj_DISAMB(1, i), traj_DISAMB(1, i) + l_axis*cos(traj_DISAMB(3, i))], [traj_DISAMB(2, i), traj_DISAMB(2, i) + l_axis*sin(traj_DISAMB(3, i))], 'Color', 'r', 'LineWidth', 2);
%     line([traj_DISAMB(1, i), traj_DISAMB(1, i) - l_axis*sin(traj_DISAMB(3, i))], [traj_DISAMB(2, i), traj_DISAMB(2, i) + l_axis*cos(traj_DISAMB(3, i))], 'Color', 'g', 'LineWidth', 2);
% end

%%
% plot_script;
% plot_goal_match;

%%
function ur = generate_autonomy(goal_index)
    global xr xg;
    ur = zeros(3,1);
    ur(1:2) = xg(1:2, goal_index) - xr(1:2);
    if norm(ur(1:2)) > 0.20
        ur(1:2) = 0.2*ur(1:2)/norm(ur(1:2));
    end
    ur(3) = generate_rotation(xg(3, goal_index), xr(3));
    
end
function uh = generate_full_uh(xg, xr) %full unnomralized uh
    global nd sparsity_factor kappa;
    uh =  zeros(nd, 1); %initialize with zero
%   uh(1:2) = xg(1:2) - xr(1:2);
    mu = xg(1:2) - xr(1:2);
    if ~any(mu)
        uh(1:2) = zeros(nd-1, 1);
    else
        uh(1:2) = randvonMisesFisherm(nd-1, 1, kappa, mu);
    end
    %create rotational component
    uh(3) = generate_rotation(xg(3), xr(3)); %essentially determines whether to tuen clockwise or anti-clockwise
    %incorporate accidental wrong direction rotation?
    if rand < sparsity_factor
        uh = zeros(nd, 1);
    end
end

function uh_r =  generate_rotation(xg, xr)
    uh_r = 0; %if if angles are same, don't turn. 
    if xr < xg
        if abs(xg-xr) <= pi
            uh_r = 0.2; %anti-clockwise
        else
            uh_r = -0.2; %clockwise
        end
    else
        if abs(xg-xr) <= pi
            uh_r = -0.2;
        else
            uh_r = 0.2;
        end
    end
end

%%
function alpha = alpha_from_confidence(goal_pg)
   global conf_thresh conf_max alpha_max
   if goal_pg <= conf_thresh
       alpha = 0.0;
   elseif goal_pg > conf_thresh && goal_pg <= conf_max
       alpha = (goal_pg - conf_thresh)*(alpha_max/(conf_max - conf_thresh));
   else
       alpha = alpha_max;
   end
end
%%
function [ best_mode ] = compute_optimal_mode_POT_SE2(xg, xr_true)
    global cm nd xr;
    Pk = zeros(nd, 1);
    Pcm = zeros(length(cm), 1); %Information density for each mode. 
    for i=1:nd-1 %first two translational dimension
        Pk(i) = abs((xg(i) - xr(i))/(xg(i) - xr_true(i)));
    end
    Pk(3) = abs(wrapToPi(xg(3) - xr(3)))/abs(wrapToPi(xg(3) - xr_true(3)));
    for i=1:length(cm)
        Pcm(i) = sum(Pk(cm{i}));
    end
    best_mode = compute_best_mode(Pcm);
end