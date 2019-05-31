function [ projected_pg ] = compute_pg_projection_SE3_human_model(curr_mode, curr_goal, projection_time, varargin )
%COMPUTE_PROJECTED_ENTROPY_R2_WITH_HUMAN_MODEL Summary of this function goes here
%   Detailed explanation goes here
global delta_t ng nd xg_T;

intent_type = varargin{1};
curr_x = varargin{2};

if strcmp(intent_type, 'conf')
    uh = generate_model_uh(curr_goal, curr_x);
    zero_dim = setdiff(1:nd,curr_mode);
    for jj=1:length(zero_dim) %mode conditioned humna policy
        uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
    end
    %normalize in SE3
    if norm(uh(1:3)) > 0.2
        uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
    end
    [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    uh(4:6) = w; %1 rad/s along w axis. 
    pg = compute_conf_SE3(uh, curr_x);
elseif strcmp(intent_type, 'dft')
    pg0 = varargin{3};
    t_stamps = 0:delta_t:projection_time;
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities. 
    pgs_project(:, 1) = pg0; %init pg0
    for i=1:length(t_stamps)-1
        curr_goal_index = datasample(find(pgs_project(:, i) == max(pgs_project(:, i))), 1);
        uh = generate_model_uh(curr_goal, curr_x); %generate human policy
        zero_dim = setdiff(1:nd,curr_mode);
        for jj=1:length(zero_dim) %mode conditioned humna policy
            uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
        end
        %normalize in SE3
        if norm(uh(1:3)) > 0.2
            uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
        end
        [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
        uh(4:6) = w; %1 rad/s along w axis. 
        %generate autonomy
%         ur = generate_autonomy(xg_T(:,:,curr_goal_index), curr_x); %autonomy command in full 2D space
%         alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
%         alpha = 0.0;
%         blend_vel = (1-alpha)*uh + alpha*ur; %blended vel
        blend_vel = uh;
        pgs_project(:, i+1) = compute_p_of_g_dft_SE3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_SE3(curr_x, blend_vel);
    end
    pg = pgs_project(:, end);
elseif strcmp(intent_type, 'bayes')
    pg0 = varargin{3};
    t_stamps = 0:delta_t:projection_time;
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities. 
    pgs_project(:, 1) = pg0; %init pg0
    for i=1:length(t_stamps)-1
        curr_goal_index = datasample(find(pgs_project(:, i) == max(pgs_project(:, i))), 1);
        uh = generate_model_uh(curr_goal, curr_x); %generate human policy
        zero_dim = setdiff(1:nd,curr_mode);
        for jj=1:length(zero_dim) %mode conditioned humna policy
            uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
        end
        %normalize in SE3
        if norm(uh(1:3)) > 0.2
            uh(1:3) = 0.2*(uh(1:3)./(norm(uh(1:3)) + realmin)); %normalize translational velocity
        end
        [w, ~] = AxisAng3(uh(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
        uh(4:6) = w; %1 rad/s along w axis. 

        %generate autonomy
%         ur = generate_autonomy(xg_T(:,:,curr_goal_index), curr_x); %autonomy command in full 2D space
%         alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
%         alpha = 0.0;
%         blend_vel = (1-alpha)*uh + alpha*ur; %blended vel
        blend_vel = uh;
        pgs_project(:, i+1) = compute_bayes_SE3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_SE3(curr_x, blend_vel);
    end
    pg = pgs_project(:, end);
end
projected_pg = pg;
end


%% uh
function uh = generate_model_uh(gT, rT) % %goal 4 by4 (gT), robot 4 by 4. 
    global nd kappa;
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
%     uh(4:6) = MatrixLog3(Rdiff); %unnormalized
%     uh = uh + normrnd(0, 0.01, nd, 1);
end
%% simple potential field type of autonomy. 
function ur = generate_autonomy(gT, rT)
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