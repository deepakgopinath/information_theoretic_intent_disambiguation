function [ best_mode ] = compute_optimal_mode_KL_SE3_goals( intent_type, xr_T, pg )
%COMPUTE_OPTIMAL_MODE_KL_SE3_GOALS Summary of this function goes here
%   Detailed explanation goes here

global cm xg_T nd ng;
candidate_uhs = zeros(nd, ng);
for i=1:ng
    candidate_uhs(:, i) = generate_full_uh(xg_T(:,:,i), xr_T);
end
EID_AR = zeros(length(cm), 1);
for i=1:length(cm)
    curr_mode = cm{i};
    if length(curr_mode) == 3
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j);  uh_t(curr_mode(2)) = candidate_uhs(curr_mode(2), j); uh_t(curr_mode(3)) = candidate_uhs(curr_mode(3), j); 
            uh_t(1:3) = 0.2*(uh_t(1:3)./(abs(uh_t(1:3)) + realmin)); %normalize translational velocity
            [w, ~] = AxisAng3(uh_t(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
            uh_t(4:6) = w;
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_kl_SE3(uh_t, intent_type, xr_T, pg);
        end
    elseif length(curr_mode) == 2
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j);  uh_t(curr_mode(2)) = candidate_uhs(curr_mode(2), j); 
            uh_t(1:3) = 0.2*(uh_t(1:3)./(abs(uh_t(1:3)) + realmin)); %normalize translational velocity
            [w, ~] = AxisAng3(uh_t(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
            uh_t(4:6) = w;
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_kl_SE3(uh_t, intent_type, xr_T, pg);
        end
            
    elseif length(curr_mode) == 1
        for j=1:size(candidate_uhs, 2)
            uh_t = zeros(nd, 1);
            uh_t(curr_mode(1)) = candidate_uhs(curr_mode(1), j); 
            uh_t(1:3) = 0.2*(uh_t(1:3)./(abs(uh_t(1:3)) + realmin)); %normalize translational velocity
            [w, ~] = AxisAng3(uh_t(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
            uh_t(4:6) = w;
            EID_AR(i) = EID_AR(i) + (1/size(candidate_uhs, 2))*compute_projected_kl_SE3(uh_t, intent_type, xr_T, pg);
        end
    end
end

best_mode = compute_best_mode(EID_AR);
end

%%
function uh = generate_full_uh(gT, rT) % %goal 4 by4 (gT), robot 4 by 4. 
    global nd;
    uh = zeros(nd,1);
    uh(1:3) = gT(1:3, 4) - rT(1:3, 4); 
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    uh(4:6) = MatrixLog3(Rdiff); %unnormalized
    uh = uh + normrnd(0, 0.01, nd, 1);
end