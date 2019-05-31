function [pg] = compute_p_of_g_dft_SE2( uh, xr, pg )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    global ng delta_t;
    tau = 5;
    h = 1/ng;
    %%%%%%%%%%%%%%%%
    %only for ad mode
%     uh = aeval(uh);
    %%%%%%%%%%%%%%%%%%
    curr_pg = compute_curr_input_SE2(uh, xr);
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

function curr_input = compute_curr_input_SE2(uh, xr)
    global ng xg;
    if size(xr, 2) > 1
        xr = xr';
    end
    curr_input = zeros(ng, 1);
    for i=1:ng %directedness based confidence function. 
        normxs = (xg(1:2, i) - xr(1:2))/(norm((xg(1:2, i) - xr(1:2))) + realmin); %ith goal position;
        normuh = uh(1:2)/(norm(uh(1:2)) +realmin); %add realmin to avoid divide by zero errors. 
        costh = dot(normxs, normuh);
        curr_input(i) = curr_input(i) + ((1 + costh)/2);
    end
    if abs(uh(3)) > 0 
        for i=1:ng
            %find shortest angle to the goal. If the sign of that angle and
            %velocity is the same, give a score of 1. else 0. For the time
            %being. We are assuming the user is operating at max velocity. 
            if xr(3) < xg(3,i)
                if abs(xg(3,i) - xr(3)) <= pi
                    diff_angle = xg(3,i) - xr(3);
                else
                    diff_angle = abs(xg(3,i) - xr(3)) - 2*pi;
                end
                if sign(uh(3)) == sign(diff_angle)
                    curr_input(i) = curr_input(i) + 1;
                else
                    curr_input(i) = curr_input(i) - 0.0;
                end
            elseif xr(3) > xg(3, i)
                if abs(xg(3,i) - xr(3)) <= pi
                    diff_angle = xg(3,i) - xr(3);
                else
                    diff_angle = 2*pi - abs(xg(3,i) - xr(3));
                end
                if sign(uh(3)) == sign(diff_angle)
                    curr_input(i) = curr_input(i) + 1;
                else
                    curr_input(i) = curr_input(i) - 0;
                end
            end

        end
    end
end

% function curr_input = compute_curr_input(uh, xr)
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