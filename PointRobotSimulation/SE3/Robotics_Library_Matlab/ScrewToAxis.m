function [screwaxis] = ScrewToAxis( q,s,h )

%Usage:
% Input - q - A point in R^3
%         s - A unit axis in R^3
%         h - pitch, always finite
% Output - ScrewAxis, 6 by 1 vector

%Example
%Input - q = [3,0,0]', s = [0,0,1] h = 2
%Output = [0,0,1,0,-3,2]'

%input validation for each one of the inputs is necessary. 
screwaxis = [];
if(size(q,2) > 1)
    q = q';
end
if(size(s,2) > 1)
    s = s';
end
if(size(q,2) > 1 || size(s, 2) > 1)
    disp('q or s is not a vector');
elseif(length(q) ~= 3 || length(s) ~= 3)
    disp('q or s is not a 3d vector');
elseif(norm(s) ~= 1)
    disp('s is not a unit vector');
elseif(sum(size(h) == [1,1]) ~= 2)
    disp('pitch is not a scalar.');
else
    screwaxis = zeros(6,1);
    screwaxis(1:3) = s;
    screwaxis(4:6) = -cross(s,q) + h*s;
end
end

