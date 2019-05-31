% clear all; clc; close all;

%% test to generate random poses for goals and plot them
% for translational range
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];
th_range = [0, 2*pi];

global num_modes cm ng nd xg_pos xg_quat xg_R xg_T xr_pos xr_quat xr_R xr_T sig delta_t num_samples conf_thresh conf_max alpha_max sparsity_factor amp_sparsity_factor kappa projection_time;
max_ng = 6;
ng = datasample(3:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 6; %num of dimensions. by definition R^3
cm_options = {{1,2,3,4,5,6}, {[1,2,3], [4,5,6]}, {[1,2],[1,3],[4,5],6}, {[1,2],[1,3],[4,5],[4,6]}};
max_nm = length(cm_options);
cm = cm_options{datasample(1:max_nm, 1)};
num_modes = length(cm);
init_mode_index = datasample(1:num_modes, 1);


%% human parameters
sparsity_factor = rand/8;
amp_sparsity_factor = rand/8; % how often the amplitude wiull be less that maximum. 
kappa = 20.0; % concentration paarameter for vonMisesFisher distribution
fprintf('The sparsity and amp factor are %f, %f\n', sparsity_factor, amp_sparsity_factor);
%%
sig = 0.01; %For Fisher information

%%
%% Projection paramaters
projection_time = 4;
delta_t = 0.1; %For compute projections. 
num_samples = 5;
total_time_steps = 120;  %with delta_t of 0.1, this amounts to 10 seconds. We will assume that "mode switches" don't take time. 

%% simulation params
mode_comp_timesteps = 10; %time step gap between optimal mode computation. delta_t*mode_comp_timesteps is the time in seconds
exit_threshold = 0.02;

%% arbitration function parameters
conf_thresh = (1.05/ng);
conf_max = (1.1/ng);
alpha_max = 0.7;

%%
%random positions for goals and robot
xg_R = zeros(3,3,ng);
xg_T = zeros(4,4,ng);
xg_pos = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(zrange) + zrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr_pos = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(zrange) + zrange(1)];

%random quaternions for goals and robot. Generate random axis. Generate
%random rotation about that axis. Combine them to produce a random
%quaternion. 
for i=1:ng
    xg_quat(:, i) = generate_rand_quaternion;
    xg_R(:,:, i) = QuatToR(xg_quat(:, i));
    xg_T(:,:,i) = rt2tr(xg_R(:,:,i), xg_pos(:,i));
end
xr_quat = generate_rand_quaternion;
xr_R = QuatToR(xr_quat);
xr_T = rt2tr(xr_R, xr_pos);
xr_T_true = xr_T;

%% PLOT GOALS AND ROBOT
% figure;
% scatter3(xg_pos(1,1:ng), xg_pos(2,1:ng), xg_pos(3,1:ng), 230, 'k', 'filled'); grid on; hold on;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% scatter3(xr_pos(1), xr_pos(2), xr_pos(3), 200, 'r', 'filled');
% offset = [-0.3, 0.3];
% line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
% line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
% line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
% axis([xrange+offset, yrange+offset, zrange+offset]);
% axis square;
% view([142,31]);
% 
% % draw body rotation frames
% l_axis = 0.1;
% trplot(xr_T, 'rgb', true, 'thick', 2.0, 'length', l_axis); hold on;
% for i=1:ng
%     trplot(xg_T(:,:,i), 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end

%% Generate the random goal towards which the simulated human would move. 
random_goal_index = randsample(ng, 1); 
random_goal_T = xg_T(:,:, random_goal_index); %homogenous matrix of the random goal
% scatter3(random_goal_T(1, 4), random_goal_T(2, 4), random_goal_T(3, 4), 230, 'm', 'filled'); grid on; hold on;

%%
intent_types = {'dft', 'conf', 'bayes'};
intent_type = intent_types{datasample(1:length(intent_types), 1)}; % or conf or bayes
% intent_type = 'dft';

%% BASELINE COMPUTATION
pgs_POT = zeros(ng, total_time_steps);
optimal_modes_POT = zeros(total_time_steps-1, 1);
alpha_POT = zeros(total_time_steps-1, 1);
uh_POT = zeros(nd, total_time_steps-1);
ur_POT = zeros(nd, total_time_steps-1);
blend_vel_POT = zeros(nd, total_time_steps-1);
curr_goal_POT = zeros(total_time_steps-1, 1);
traj_POT = cell(total_time_steps, 1);
traj_POT{1} = xr_T; 
pgs_POT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection
current_optimal_mode_POT_index = init_mode_index;
current_optimal_mode_POT = cm{current_optimal_mode_POT_index};


for i=1:total_time_steps-1
    if compute_dist_to_goal(random_goal_T, xr_T_true)
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_POT = datasample(find(pgs_POT(:, i) == max(pgs_POT(:, i))), 1);
    curr_goal_POT(i) = curr_goal_index_POT;
    %compute the optimal mode. store it away. 
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_POT_index = compute_optimal_mode_POT_SE3(xg_T(:, :, curr_goal_index_POT), xr_T_true); 
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
    %possible motion in the current mode towards the specified goal". 
    uh = generate_full_uh(random_goal_T, xr_T);
    zero_dim = setdiff(1:nd,current_optimal_mode_POT);
    for jj=1:length(zero_dim) %zero out the non-accessible dimensions. 
        uh(zero_dim(jj)) = 0;
    end
    if norm(uh(1:3)) > 0.2
        uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
    end
    [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    uh(4:6) = w; %1 rad/s along w axis. 
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end


    % autonomy
    ur = generate_full_autonomy(xg_T(:,:,curr_goal_index_POT), xr_T);
    alpha_POT(i) = alpha_from_confidence(pgs_POT(curr_goal_index_POT, i));
    blend_vel = (1-alpha_POT(i))*uh + alpha_POT(i)*ur; %blended vel
    uh_POT(:, i) = uh; ur_POT(:, i) = ur; blend_vel_POT(:, i) = blend_vel;
    
     %Evolve probabilities
    if strcmp(intent_type, 'dft')
        pgs_POT(:, i+1) = compute_p_of_g_dft_SE3(uh, xr_T, pgs_POT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_POT(:, i+1) = compute_bayes_SE3(uh, xr_T, pgs_POT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_POT(:, i+1) = compute_conf_SE3(uh, xr_T);
    end
    %simulate the the full 6D kinematics. 
    xr_T = sim_kinematics_SE3(xr_T, blend_vel);
    traj_POT{i+1} = xr_T;
    optimal_modes_POT(i) = current_optimal_mode_POT_index; 
end



%% ENTROPY BASED COMPUTATION
fprintf("SIMULATION USING ENTROPY INFORMATION \n");
xr_T = xr_T_true;
pgs_ENT = zeros(ng, total_time_steps);
optimal_modes_ENT = zeros(total_time_steps-1, 1);
alpha_ENT = zeros(total_time_steps-1, 1);
uh_ENT = zeros(nd, total_time_steps-1);
ur_ENT = zeros(nd, total_time_steps-1);
blend_vel_ENT = zeros(nd, total_time_steps-1);
curr_goal_ENT = zeros(total_time_steps-1, 1);
%Simulate directly in 4 by 4 homogenous transformation matrix space. 
traj_ENT = cell(total_time_steps, 1);
traj_ENT{1} = xr_T; 
pgs_ENT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_ENT_index = init_mode_index;
current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index};

for i=1:total_time_steps-1
    if compute_dist_to_goal(random_goal_T, xr_T_true)
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_ENT = datasample(find(pgs_ENT(:, i) == max(pgs_ENT(:, i))), 1);
    curr_goal_ENT(i) = curr_goal_index_ENT;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_ENT_index = compute_optimal_mode_ENT_SE3_human_model(intent_type, xr_T, pgs_ENT(:, i)); 
        if length(current_optimal_mode_ENT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index(1)}; %pick the first one. 
            current_optimal_mode_ENT_index = current_optimal_mode_ENT_index(1); %update the index. 
        else
            if current_optimal_mode_ENT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index}; 
            end
        end
    end
    %generate human control command
    uh = generate_full_uh(random_goal_T, xr_T);
    zero_dim = setdiff(1:nd,current_optimal_mode_ENT);
    for jj=1:length(zero_dim) %zero out the non-accessible dimensions. 
        uh(zero_dim(jj)) = 0;
    end
    if norm(uh(1:3)) > 0.2
        uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
    end
    [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    uh(4:6) = w; %1 rad/s along w axis. 
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    %autonomy
    ur = generate_full_autonomy(xg_T(:,:,curr_goal_index_ENT), xr_T);
    alpha_ENT(i) = alpha_from_confidence(pgs_ENT(curr_goal_index_ENT, i));
    blend_vel = (1-alpha_ENT(i))*uh + alpha_ENT(i)*ur; %blended vel
    uh_ENT(:, i) = uh; ur_ENT(:, i) = ur; blend_vel_ENT(:, i) = blend_vel;
     %Evolve probabilities
    if strcmp(intent_type, 'dft')
        pgs_ENT(:, i+1) = compute_p_of_g_dft_SE3(uh, xr_T, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_ENT(:, i+1) = compute_bayes_SE3(uh, xr_T, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_ENT(:, i+1) = compute_conf_SE3(uh, xr_T);
    end
    %simulate the the full 6D kinematics. 
    xr_T = sim_kinematics_SE3(xr_T, blend_vel);
    traj_ENT{i+1} = xr_T;
    optimal_modes_ENT(i) = current_optimal_mode_ENT_index; 
end
%%
%% KL based
fprintf("SIMULATION USING KL DIVERGENCE\n");
xr_T = xr_T_true;
pgs_KL = zeros(ng, total_time_steps);
optimal_modes_KL = zeros(total_time_steps-1, 1);
alpha_KL = zeros(total_time_steps-1, 1);
uh_KL = zeros(nd, total_time_steps-1);
ur_KL = zeros(nd, total_time_steps-1);
blend_vel_KL = zeros(nd, total_time_steps-1);
curr_goal_KL = zeros(total_time_steps-1, 1);
%Simulate directly in 4 by 4 homogenous transformation matrix space. 
traj_KL = cell(total_time_steps, 1);
traj_KL{1} = xr_T; 
% traj_KL(:, 1) = xr; %(x,y,theta)
pgs_KL(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_KL_index = init_mode_index;
current_optimal_mode_KL = cm{current_optimal_mode_KL_index};

for i=1:total_time_steps-1
    if compute_dist_to_goal(random_goal_T, xr_T_true)
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_KL = datasample(find(pgs_KL(:, i) == max(pgs_KL(:, i))), 1);
    curr_goal_KL(i) =  curr_goal_index_KL;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_KL_index = compute_optimal_mode_KL_SE3_human_model(intent_type, xr_T, pgs_KL(:, i)); 
        if length(current_optimal_mode_KL_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_KL = cm{current_optimal_mode_KL_index(1)}; %pick the first one. 
            current_optimal_mode_KL_index = current_optimal_mode_KL_index(1); %update the index. 
        else
            if current_optimal_mode_KL_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_KL = cm{current_optimal_mode_KL_index}; 
            end
        end
    end
    %generate human control command
    uh = generate_full_uh(random_goal_T, xr_T);
    zero_dim = setdiff(1:nd,current_optimal_mode_KL);
    for jj=1:length(zero_dim) %zero out the non-accessible dimensions. 
        uh(zero_dim(jj)) = 0;
    end
    if norm(uh(1:3)) > 0.2
        uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
    end
    
    [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    uh(4:6) = w; %1 rad/s along w axis. 
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end
    
    ur = generate_full_autonomy(xg_T(:,:,curr_goal_index_KL), xr_T);
    alpha_KL(i) = alpha_from_confidence(pgs_KL(curr_goal_index_KL, i));
    blend_vel = (1-alpha_KL(i))*uh + alpha_KL(i)*ur; %blended vel
    uh_KL(:, i) = uh; ur_KL(:, i) = ur; blend_vel_KL(:, i) = blend_vel;
    %Evolve probabilities
    if strcmp(intent_type, 'dft')
        pgs_KL(:, i+1) = compute_p_of_g_dft_SE3(uh, xr_T, pgs_KL(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_KL(:, i+1) = compute_bayes_SE3(uh, xr_T, pgs_KL(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_KL(:, i+1) = compute_conf_SE3(uh, xr_T);
    end
    %simulate the the full 6D kinematics. 
    xr_T = sim_kinematics_SE3(xr_T, blend_vel);
    traj_KL{i+1} = xr_T;
    optimal_modes_KL(i) = current_optimal_mode_KL_index; 
end

% 
% %% FI based
% fprintf("SIMULATION USING FISHER INFORMATION \n");
% xr_T = xr_T_true;
% pgs_FI = zeros(ng, total_time_steps);
% optimal_modes_FI = zeros(total_time_steps-1, 1);
% alpha_FI = zeros(total_time_steps-1, 1);
% uh_FI = zeros(nd, total_time_steps-1);
% ur_FI = zeros(nd, total_time_steps-1);
% blend_vel_FI = zeros(nd, total_time_steps-1);
% curr_goal_FI = zeros(total_time_steps-1, 1);
% %Simulate directly in 4 by 4 homogenous transformation matrix space. 
% traj_FI = cell(total_time_steps, 1);
% traj_FI{1} = xr_T; 
% % traj_FI(:, 1) = xr; %(x,y,theta)
% pgs_FI(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
% %internally projection
% 
% current_optimal_mode_FI_index = 1;
% current_optimal_mode_FI = cm{current_optimal_mode_FI_index};
% 
% for i=1:total_time_steps
%     curr_goal_index_FI = datasample(find(pgs_FI(:, i) == max(pgs_FI(:, i))), 1);
%     curr_goal_FI(i) =  curr_goal_index_FI;
%     if mod(i-1, mode_comp_timesteps) == 0
%         current_optimal_mode_FI_index = compute_optimal_mode_FI_SE3(intent_type, xr_T, pgs_FI(:, i)); 
%         if length(current_optimal_mode_FI_index) > 1 %when there are equivalent modes. 
%             current_optimal_mode_FI = cm{current_optimal_mode_FI_index(1)}; %pick the first one. 
%             current_optimal_mode_FI_index = current_optimal_mode_FI_index(1); %update the index. 
%         else
%             if current_optimal_mode_FI_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
%                 current_optimal_mode_FI = cm{current_optimal_mode_FI_index}; 
%             end
%         end
%     end
%     %generate human control command
%     uh = generate_full_uh(random_goal_T, xr_T);
%     zero_dim = setdiff(1:nd,current_optimal_mode_FI);
%     for jj=1:length(zero_dim) %zero out the non-accessible dimensions. 
%         uh(zero_dim(jj)) = 0;
%     end
%     uh(1:3) = 0.2*(uh(1:3)./(abs(uh(1:3)) + realmin)); %normalize translational velocity
%     [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
%     uh(4:6) = w; %1 rad/s along w axis. 
%     ur = generate_full_autonomy(xg_T(:,:,curr_goal_index_FI), xr_T);
%     alpha_FI(i) = alpha_from_confidence(pgs_FI(curr_goal_index_FI, i));
%     blend_vel = (1-alpha_FI(i))*uh + alpha_FI(i)*ur; %blended vel
%     uh_FI(:, i) = uh; ur_FI(:, i) = ur; blend_vel_FI(:, i) = blend_vel;
%     %Evolve probabilities
%     if strcmp(intent_type, 'dft')
%         pgs_FI(:, i+1) = compute_p_of_g_dft_SE3(uh, xr_T, pgs_FI(:, i));
%     elseif strcmp(intent_type, 'bayes')
%         pgs_FI(:, i+1) = compute_bayes_SE3(uh, xr_T, pgs_FI(:, i));
%     elseif strcmp(intent_type, 'conf')
%         pgs_FI(:, i+1) = compute_conf_SE3(uh, xr_T);
%     end
%     %simulate the the full 6D kinematics. 
%     xr_T = sim_kinematics_SE3(xr_T, blend_vel);
%     traj_FI{i+1} = xr_T;
%     optimal_modes_FI(i) = current_optimal_mode_FI_index; 
% end

%%
%% DISAMB based
fprintf("SIMULATION USING DISAMB METRIC \n");
xr_T = xr_T_true;
pgs_DISAMB = zeros(ng, total_time_steps);
optimal_modes_DISAMB = zeros(total_time_steps-1, 1);
alpha_DISAMB = zeros(total_time_steps-1, 1);
uh_DISAMB = zeros(nd, total_time_steps-1);
ur_DISAMB = zeros(nd, total_time_steps-1);
blend_vel_DISAMB = zeros(nd, total_time_steps-1);
curr_goal_DISAMB = zeros(total_time_steps-1, 1);
%Simulate directly in 4 by 4 homogenous transformation matrix space. 
traj_DISAMB = cell(total_time_steps, 1);
traj_DISAMB{1} = xr_T; 
% traj_DISAMB(:, 1) = xr; %(x,y,theta)
pgs_DISAMB(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_DISAMB_index = init_mode_index;
current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index};

for i=1:total_time_steps-1
    if compute_dist_to_goal(random_goal_T, xr_T_true)
        fprintf('Within 10 percent of the original separation from goal. Stopping simulation\n');
        break;
    end
    curr_goal_index_DISAMB = datasample(find(pgs_DISAMB(:, i) == max(pgs_DISAMB(:, i))), 1);
    curr_goal_DISAMB(i) = curr_goal_index_DISAMB;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_DISAMB_index = compute_optimal_mode_DISAMB_SE3(intent_type, xr_T, pgs_DISAMB(:, i)); 
        if length(current_optimal_mode_DISAMB_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index(1)}; %pick the first one. 
            current_optimal_mode_DISAMB_index = current_optimal_mode_DISAMB_index(1); %update the index. 
        else
            if current_optimal_mode_DISAMB_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index}; 
            end
        end
    end
    %generate human control command
    uh = generate_full_uh(random_goal_T, xr_T);
    zero_dim = setdiff(1:nd,current_optimal_mode_DISAMB);
    for jj=1:length(zero_dim) %zero out the non-accessible dimensions. 
        uh(zero_dim(jj)) = 0;
    end
    if norm(uh(1:3)) > 0.2
        uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
    end
    [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    uh(4:6) = w; %1 rad/s along w axis. 
    
    if rand < amp_sparsity_factor
        uh = rand*uh;
    end

    %autonomy
    ur = generate_full_autonomy(xg_T(:,:,curr_goal_index_DISAMB), xr_T);
    alpha_DISAMB(i) = alpha_from_confidence(pgs_DISAMB(curr_goal_index_DISAMB, i));
    blend_vel = (1-alpha_DISAMB(i))*uh + alpha_DISAMB(i)*ur; %blended vel
    uh_DISAMB(:, i) = uh; ur_DISAMB(:, i) = ur; blend_vel_DISAMB(:, i) = blend_vel;
    %Evolve probabilities
    if strcmp(intent_type, 'dft')
        pgs_DISAMB(:, i+1) = compute_p_of_g_dft_SE3(uh, xr_T, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_DISAMB(:, i+1) = compute_bayes_SE3(uh, xr_T, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_DISAMB(:, i+1) = compute_conf_SE3(uh, xr_T);
    end
    %simulate the the full 6D kinematics. 
    xr_T = sim_kinematics_SE3(xr_T, blend_vel);
    traj_DISAMB{i+1} = xr_T;
    optimal_modes_DISAMB(i) = current_optimal_mode_DISAMB_index; 
end

%% plot the trajectory with the rotation frame. 
% hold on;
% pause('on');
% skip_step = 3;

% for i=1:skip_step:total_time_steps
%     if isempty(traj_POT{i})
%        break;
%     end
%     curr_x = traj_POT{i}(1:3, 4);
%     scatter3(curr_x(1), curr_x(2), curr_x(3), 'r', 'filled');
%     trplot(traj_POT{i}, 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end
% hold on;
% for i=1:skip_step:total_time_steps
%     if isempty(traj_ENT{i})
%        break;
%     end
%     curr_x = traj_ENT{i}(1:3, 4);
%     scatter3(curr_x(1), curr_x(2), curr_x(3), 'r', 'filled');
%     trplot(traj_ENT{i}, 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end
% hold on;
% for i=1:skip_step:total_time_steps
%     if isempty(traj_KL{i})
%        break;
%     end
%     curr_x = traj_KL{i}(1:3, 4);
%     scatter3(curr_x(1), curr_x(2), curr_x(3), 'b', 'filled');
%     trplot(traj_KL{i}, 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end
% 
% hold on;
% for i=1:skip_step:total_time_steps
%     curr_x = traj_FI{i}(1:3, 4);
%     scatter3(curr_x(1), curr_x(2), curr_x(3), 'b', 'filled');
%     trplot(traj_FI{i}, 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end

% hold on;
% for i=1:skip_step:total_time_steps
%     if isempty(traj_DISAMB{i})
%        break;
%     end
%     curr_x = traj_DISAMB{i}(1:3, 4);
%     scatter3(curr_x(1), curr_x(2), curr_x(3), 'g', 'filled');
%     trplot(traj_DISAMB{i}, 'rgb', true, 'thick', 2.0, 'length', l_axis);
% end

%% plot script for goal inference and mode switches
% plot_script;
% plot_goal_match;


%% UTILITY FUNCTIONS
function uh = generate_full_uh(gT, rT) % %goal 4 by4 (gT), robot 4 by 4. 
    global nd kappa sparsity_factor;
    uh = zeros(nd,1);
    mu_T = gT(1:3, 4) - rT(1:3, 4); 
    if ~any(mu_T)
        uh(1:3) = zeros(nd-3, 1);
    else
        uh(1:3) = randvonMisesFisherm(nd-3, 1, kappa, mu_T); 
    end
    
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    mu_R = MatrixLog3(Rdiff); %unnormalized
    if ~any(mu_R)
        uh(4:6) = zeros(nd-3, 1);
    else
        uh(4:6) = randvonMisesFisherm(nd-3, 1, 2*kappa, mu_R); %more concentrated dispoersion for rotation component
    end
    if rand  < sparsity_factor
        uh = zeros(nd, 1);
    end
end

function ur = generate_full_autonomy(gT, rT)
    global nd;
    ur = zeros(nd,1);
    ur(1:3) = gT(1:3, 4) - rT(1:3, 4);
    if norm(ur(1:3)) > 0.2
        ur(1:3) = 0.2*ur(1:3)/norm(ur(1:3));
    end
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    ur(4:6) = MatrixLog3(Rdiff); %unnormalized
    [w, ~] = AxisAng3(ur(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    ur(4:6) = w; %1 rad/s along w axis. 
end
%%
function q = generate_rand_quaternion
    th_range = [0, 2*pi];
    w = rand(3,1); %create random axis
    w = w/norm(w); %unit vector - axis of rotation
    theta = rand(1,1)*range(th_range) + th_range(1); %creat random angle of rotation. 
    r = w*theta;    
    q = AxisAng_Quat(r); %returns column vector of unit quaternion
end
%% arbitration function

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
function [ best_mode ] = compute_optimal_mode_POT_SE3(xg_T, xr_T_true)
    global cm nd xr_T;
    Pk = zeros(nd, 1);
    Pcm = zeros(length(cm), 1); %Information density for each mode. 
    for i=1:nd-3 %only translational dimensions. 
        Pk(i) = abs((xg_T(i, 4) - xr_T(i, 4))/(xg_T(i, 4) - xr_T_true(i, 4)));
    end
    Rg = xg_T(1:3, 1:3); Rr = xr_T(1:3, 1:3); Rr_true = xr_T_true(1:3, 1:3);
    Rdiff_w = Rg*(Rr^-1); Rdiff_w_true = Rg*(Rr_true^-1);
    
    %THE FOLLOWING IS SUSPICIOUS. WATCH OUT!!!
    [w,~] = AxisAng3(MatrixLog3(Rdiff_w));
    [w_true, ~] = AxisAng3(MatrixLog3(Rdiff_w_true));
    Pk(4:end) = w./w_true; %How aligned are the two angle axis vectors? 
    for i=1:length(cm)
        Pcm(i) = sum(Pk(cm{i}));
    end
    %decide the fractional potential for the rotational dimensions. 
    best_mode = compute_best_mode(Pcm);
end

function isClose = compute_dist_to_goal(xg_T, xr_T_true)
    global xr_T; %current position. 
    exit_threshold = 0.02; ori_threshold = 0.05;
    fracPosDiff = norm(xr_T(1:3, 4) - xg_T(1:3, 4))/norm(xr_T_true(1:3, 4) - xg_T(1:3, 4)); 
    Rg = xg_T(1:3, 1:3); Rr = xr_T(1:3, 1:3);
    Rdiff_w = Rg*(Rr^-1); 
    [~,theta] = AxisAng3(MatrixLog3(Rdiff_w)); %If Rg and Rr are close to each other, Rdiff wil l be close to identity and theta will be close to 0. 
    if fracPosDiff <= exit_threshold && theta < ori_threshold
        isClose = true;
    else
        isClose = false;
    end
end