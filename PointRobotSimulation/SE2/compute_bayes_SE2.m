function [ posterior ] = compute_bayes_SE2( uh, xr, prior )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global ng
    ll = likelihood_func_trans(uh(1:2), xr(1:2)).*likelihood_func_rot(uh(3), xr(3));
    prior = prior + 0.01*rand(ng, 1); %to avoid collapse of posterior. 
    prior = prior/sum(prior);
    posterior = ll.*prior;
    posterior = posterior/sum(posterior); %make sure that the posterior is always normalized. 
end

%%
function likelihood = likelihood_func_trans(uh, xr)
    global ng xg;
    likelihood = zeros(ng, 1);
    if norm(uh)~=0
        uh = uh/norm(uh);
    else
        likelihood = ones(ng, 1); %if zero velocity don';t change inference. best way to do it is to have the likelihood to be the same for all goals. 
        return;
    end
    for i=1:ng
        dir_vec = xg(1:2, i) - xr; %vector joining the robot position and the goal_x. 
        dir_vec = dir_vec/norm(dir_vec); %as long as xr is not xg, this would be nonzero;
        likelihood(i) = p_of_u_given_g(uh, dir_vec);
    end
end

function [q] = p_of_u_given_g(uh, dir_vec) %assumes that the uh that is expected to see for a goal is the one which is directly pointing at it. 
%     q = prob(uh(1) - dir_vec(1))*prob(uh(2) - dir_vec(2))*prob(uh(3) - dir_vec(3)); %p(ux, uy, uz) = p(ux)p(uy)p(uz);
    ang = acos(dot(uh, dir_vec));
    q = prob(ang);
end

%%
function likelihood3 = likelihood_func_rot(uh_r, xr_r) %likelihood for the rotational compoinent. If the rotational velocity matches 
    global ng xg;
    likelihood3 = zeros(ng, 1);
    if uh_r ~=0
        uh_r = uh_r/abs(uh_r);
    else
        likelihood3 = (1/ng)*ones(ng, 1); %if uh3 is zero the likelhood associated with the goals i
        return;
    end
    for i=1:ng
         if xr_r < xg(3,i)
            if abs(xg(3,i) - xr_r) <= pi
                diff_angle = xg(3,i) - xr_r;
            else
                diff_angle = abs(xg(3,i) - xr_r) - 2*pi;
            end
            if sign(uh_r) == sign(diff_angle)
                likelihood3(i) = 0.9;
            else
                likelihood3(i) = 0.1;
            end
        elseif xr_r > xg(3, i)
            if abs(xg(3,i) - xr_r) <= pi
                diff_angle = xg(3,i) - xr_r;
            else
                diff_angle = 2*pi - abs(xg(3,i) - xr_r);
            end
            if sign(uh_r) == sign(diff_angle)
                likelihood3(i) = 0.9;
            else
                likelihood3(i) = 0.1;
            end
         elseif xr_r == xg(3, i) %angles are equal but uh3 is not zero. person is trying to move away from a goal position
             likelihood3(i) = 0.1;
         end
    end
    
end

%%
function p = prob(a) %zero mean gaussian, 
    sigma = 1.5*pi; %for the time being use this std dev
    p = (1/sqrt(2*pi*sigma^2))*exp((-1/2.0)*(a^2/sigma^2));
end