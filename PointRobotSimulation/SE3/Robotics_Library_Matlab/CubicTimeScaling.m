function [ s_t ] = CubicTimeScaling(T, t)
%Input - T - Total time for the trajectory
%        t - time at which the path is queried. has to be between 0 and T.t
%        can be a scalar of a vector. 
%        

% Output - Value of s[t] at t, which will be between 0 and 1. Same
% dimensionality as t. 


% t can be vector of time values, a matrix or a scalar
if(size(t, 2) > 1)
    t = t';
end
s_t = [];
if (sum(t > T) > 0 || sum(t < 0) > 0)
    disp('the values of t has to be 0 <= t <= T');
else
    s_t = (3/(T^2))*(t.^2) + (-2/(T^3))*(t.^3);
end
end

