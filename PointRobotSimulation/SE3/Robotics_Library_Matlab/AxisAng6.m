function [ S, theta ] = AxisAng6( stheta )

%Usage:
% Input - Exponential coordinate for rigid body motion, 6 by 1
% Output - Screw Axis (6 by 1) and the distance (theta) travelled along the screw axis


S = [];
theta = [];
if(size(stheta,2) > 1) %Make it a column vector if a row vector
    stheta = stheta';
end

if(size(stheta,2) > 1) % if a matrix
    disp('stheta is not a vector');
elseif(size(stheta, 1) ~= 6)
    disp('stheta is not a 6d vector');
else
    if(sum(stheta == [0,0,0,0,0,0]') == 6) %trivial case
        S = zeros(6,1);
        theta = 0;
    elseif(sum(stheta(1:3) == [0,0,0]') ~= 3) %omega is not zero. 
        S = stheta/norm(stheta(1:3)); % if omega is not 0 normalize using omega else normalize using v
        theta = norm(stheta(1:3));
    else
        S = stheta/norm(stheta(4:6));
        theta = norm(stheta(4:6));
    end
end
end

