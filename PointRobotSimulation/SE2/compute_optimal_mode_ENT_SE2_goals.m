function [ best_mode ] = compute_optimal_mode_ENT_SE2_goals( intent_type, xr_T, pg )
%COMPUTE_OPTIMAL_MODE_ENT_SE2_GOALS Summary of this function goes here
%   Detailed explanation goes here

global cm xg ng nd;
candidate_uhs = zeros(nd, ng);
for i=1:ng
    candidate_uhs(:, i) = generate_full_uh(xg(:, i), xr_T); 
end

EID_AR = zeros(length(cm), 1); %Information density for each mode. 
for i=1:length(cm)
    curr_mode = cm{i};
    if length(curr_mode) == 3
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j);  uh_t(curr_mode(2)) = candidate_uhs(curr_mode(2), j); uh_t(curr_mode(3)) = candidate_uhs(curr_mode(3), j); 
            uh_t = 0.2*uh_t/norm(uh_t);
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_entropy_SE2(uh_t, intent_type, xr_T, pg);
        end
    elseif length(curr_mode) == 2
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j);  uh_t(curr_mode(2)) = candidate_uhs(curr_mode(2), j); 
            uh_t = 0.2*uh_t/norm(uh_t);
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_entropy_SE2(uh_t, intent_type, xr_T, pg);
        end
            
    elseif length(curr_mode) == 1
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j); 
            uh_t = 0.2*uh_t/norm(uh_t);
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_entropy_SE2(uh_t, intent_type, xr_T, pg);
        end
    end
    
end
best_mode = compute_best_mode(EID_AR);


end

%% generate u_h for goals

function uh = generate_full_uh(xg, xr) %full unnomralized uh
    global nd;
    uh = [0,0,0]'; %initialize with zero
    uh(1:2) = xg(1:2) - xr(1:2);
%     uh(1:2) = 0.2*(uh(1:2)./(abs(uh(1:2)) + realmin)); %make the dimensions at max. for translational velocity
    %create rotational component
    uh(3) = generate_rotation(xg(3), xr(3)); %essentially determines whether to tuen clockwise or anti-clockwise
    uh = uh + normrnd(0, 0.01, nd, 1);
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