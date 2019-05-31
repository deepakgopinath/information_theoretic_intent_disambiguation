function [ w, theta ] = AxisAng3( r )


% Usage
% Input - r = exponential coordinates for rotation r = w*theta
% Output - The unit vector w and the theta

%Example
% Input - r = [pi/4, 0, 0]
% Output [w, theta] = [1,0,0]' and 0.784

w = [];
theta = [];
if(size(r,2) > 1) %Make it a column vector if a row vector
    r = r';
end

if(size(r,2) > 1) % if a matrix
    disp('r is not a vector');
elseif(size(r, 1) ~= 3)
    disp('r is not a 3d vector');
else
    if(sum(r == zeros(3,1)) == 3)
        w = [0,0,0]';
        theta = 0;
    else
        w = r/norm(r); %unit vector omega
        theta = norm(r);
    end
end

