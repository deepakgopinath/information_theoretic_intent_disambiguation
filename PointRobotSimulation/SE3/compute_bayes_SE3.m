function [ posterior ] = compute_bayes_SE3( uh, xr_T, prior )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global ng;
    ll = likelihood_func_trans(uh(1:3), xr_T).*likelihood_func_rot(uh(4:6), xr_T);
    
    prior = prior + 0.01*rand(ng, 1); %to avoid collapse of posterior. 
    prior = prior/(sum(prior) + realmin);
    posterior = ll.*prior;
    posterior = posterior/(sum(posterior) +realmin); %make sure that the posterior is always normalized. 
end

function likelihood_t = likelihood_func_trans(uh_t, xr_T)
    global ng xg_T;
    likelihood_t = zeros(ng, 1);
    if norm(uh_t)~=0
        uh_t = uh_t/(norm(uh_t)+realmin);
    else
        likelihood_t = ones(ng, 1);
        return;
    end
    for i=1:ng
        dir_vec = xg_T(1:3,4,i) - xr_T(1:3, 4); %vector joining the robot position and the goal_x. - translational
        dir_vec = dir_vec/(norm(dir_vec) + realmin); %as long as xr is not xg, this would be nonzero;
        likelihood_t(i) = p_of_u_given_g(uh_t, dir_vec);
    end
end

%%
function likelihood_r = likelihood_func_rot(uh_r, xr_T)
    global ng xg_T;
    likelihood_r = zeros(ng, 1);
    if norm(uh_r) ~= 0 
        uh_r = uh_r/norm(uh_r);
    else
        likelihood_r = ones(ng, 1);
        return;
    end
    for i=1:ng
        goal_vel = generate_R_autonomy(xg_T(:,:,i), xr_T);
        if norm(goal_vel(4:6)) > 0.000
            rot_vel = goal_vel(4:6);
            rot_vel = rot_vel/(norm(rot_vel) + realmin);
            likelihood_r(i) = p_of_u_given_g(uh_r, rot_vel);
        end
        
    end
end
%%
function [q] = p_of_u_given_g(uh, dir_vec) %assumes that the uh that is expected to see for a goal is the one which is directly pointing at it. 
%     q = prob(uh(1) - dir_vec(1))*prob(uh(2) - dir_vec(2))*prob(uh(3) - dir_vec(3)); %p(ux, uy, uz) = p(ux)p(uy)p(uz);
    ang = acos(round(dot(uh, dir_vec), 5));
    q = prob(ang);
    if any(imag(q))
        disp(q);
    end
end

function p = prob(a) %zero mean gaussian, 
    sigma = pi/2; %for the time being use this std dev
    p = (1/sqrt(2*pi*sigma^2))*exp((-1/2.0)*(a^2/sigma^2));
    
end

%%
function ur = generate_R_autonomy(gT, rT)
    ur = zeros(6,1);
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    ur(4:6) = MatrixLog3(Rdiff); %unnormalized
    [w, ~] = AxisAng3(ur(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    ur(4:6) = w; %1 rad/s along w axis. 
end