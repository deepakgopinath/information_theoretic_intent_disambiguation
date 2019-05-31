function [pg] = compute_p_of_g_dft_SE3( uh, xr_T, pg )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    global ng delta_t;
    tau = 5;
    h = 1/ng;
    %%%%%%%%%%%%%%%%
    %only for ad mode
%     uh = aeval(uh);
    %%%%%%%%%%%%%%%%%%
    curr_pg = compute_curr_input_SE3(uh, xr_T);
    lambda = 0*ones(ng,ng);
    lambda(1: ng+1: ng*ng) = 0;
    lambda = 1*eye(ng) + lambda;
    dpgdt = (-1/tau)*eye(ng)*pg + (h/tau)*ones(ng,1) + lambda*sigmoid(curr_pg); %ODE - Dynamics neural field. 
    pg = pg + dpgdt*delta_t; %Euler integration;
    pg(pg <=0) = realmin;
    pg = pg/sum(pg); %normalize, so that the inidividual values are alwyas between 0 and 1. 
end

function out = sigmoid(u)
    out = 1./(1 + exp(-u));
    out = out - 0.5;
end

function curr_input = compute_curr_input_SE3(uh, xr_T)
    global ng xg_T;
    xr_pos = xr_T(1:3, 4); %robot position

    curr_input = zeros(ng, 1); %initialize
    if norm(uh(1:3)) > 0.000001 % if translation uh is nonzero. directedness
        for i=1:ng
            curr_gT = xg_T(:,:,i);
            xg_pos = curr_gT(1:3, 4);
            normxs = (xg_pos - xr_pos)/(norm(xg_pos - xr_pos) + realmin);
            normuh = uh(1:3)/(norm(uh(1:3)) + realmin);
            costh = dot(normxs, normuh);
            curr_input(i) = (1 + costh)/2;
        end
    end
    if norm(uh(4:6)) > 0.000001 %rotational agreement. 
        for i=1:ng
            goal_vel = generate_R_autonomy(xg_T(:,:,i), xr_T);
            if norm(goal_vel(4:6)) > 0.0001
                rot_vel = goal_vel(4:6);
                rot_align = dot(uh(4:6), rot_vel)/(norm(uh(4:6)) * norm(rot_vel) +  realmin);
                rot_align = (1 + rot_align)*0.5;
                curr_input(i) = curr_input(i) + rot_align;
            end
        end
    end
end

function ur = generate_R_autonomy(gT, rT)
    ur = zeros(6,1);
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    ur(4:6) = MatrixLog3(Rdiff); %unnormalized
    [w, ~] = AxisAng3(ur(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    ur(4:6) = w; %1 rad/s along w axis. 
end
% 
% function curr_input = compute_curr_input_SE3(uh, xr)
%     global ng xg;
%     if size(xr, 2) > 1
%         xr = xr';
%     end
%     curr_input = zeros(ng, 1);
%     for i=1:ng %directedness based confidence function. 
%         normxs = (xg(:, i) - xr)/(norm((xg(:, i) - xr)) + realmin); %ith goal position;
%         normuh = uh/(norm(uh) +realmin); %add realmin to avoid divide by zero errors. 
%         costh = dot(normxs, normuh);
%         curr_input(i) = (1 + costh)/2;
%     end
% end