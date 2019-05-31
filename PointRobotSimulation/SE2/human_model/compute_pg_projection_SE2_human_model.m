function [ projected_pg ] = compute_pg_projection_SE2_human_model(curr_mode, curr_goal, projection_time, varargin )
%COMPUTE_PROJECTED_ENTROPY_R2_WITH_HUMAN_MODEL Summary of this function goes here
%   Detailed explanation goes here
global delta_t ng nd;

intent_type = varargin{1};
curr_x = varargin{2};

if strcmp(intent_type, 'conf')
    uh = generate_model_uh(curr_goal, curr_x);
    zero_dim = setdiff(1:nd,curr_mode);
    for jj=1:length(zero_dim) %mode conditioned humna policy
        uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
    end
    if norm(uh(1:2)) > 0.2
        uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
    end
    pg = compute_conf_SE2(uh, curr_x);
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
        if norm(uh(1:2)) > 0.2
            uh(1:2) = 0.2*(uh(1:2)./(norm(uh(1:2)) + realmin));
        end
        %generate autonomy
%         ur = generate_autonomy(curr_goal_index, curr_x); %autonomy command in full 2D space
%         alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
%         alpha = 0.0;
        
        blend_vel = uh; %blended vel
        pgs_project(:, i+1) = compute_p_of_g_dft_SE2(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_SE2(curr_x, blend_vel);
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
        if norm(uh) > 0.2
            uh = 0.2*(uh./(norm(uh) + realmin));
        end
        %generate autonomy
%         ur = generate_autonomy(curr_goal_index, curr_x); %autonomy command in full 2D space
%         alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
%         alpha = 0.0;
        blend_vel = uh; %blended vel
        pgs_project(:, i+1) = compute_bayes_SE2(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_SE2(curr_x, blend_vel);
    end
    pg = pgs_project(:, end);
end
projected_pg = pg;
end

%% simple potential field type of autonomy. 
function ur = generate_autonomy(goal_index, curr_x)
    global xg;
    ur = zeros(3,1);
    ur(1:2) = xg(1:2, goal_index) - curr_x(1:2);
    if norm(ur(1:2)) > 0.20
        ur(1:2) = 0.2*ur(1:2)/norm(ur(1:2));
    end
    ur(3) = generate_rotation(xg(3, goal_index), curr_x(3));
end
%% uh
function uh = generate_model_uh(xg, xr) %full unnomralized uh
    global nd kappa;
    uh = [0,0,0]'; %initialize with zero
    mu = xg(1:2) - xr(1:2);
    if ~any(mu)
        uh(1:2) = zeros(nd-1, 1);
    else
        uh(1:2) = randvonMisesFisherm(nd-1, 1, kappa, mu);
    end
%     uh(1:2) = randvonMisesFisherm(nd-1, 1, kappa, mu);
%     uh(1:2) = 0.2*(uh(1:2)./(abs(uh(1:2)) + realmin)); %make the dimensions at max. for translational velocity
    %create rotational component
    uh(3) = generate_rotation(xg(3), xr(3)); %essentially determines whether to tuen clockwise or anti-clockwise
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