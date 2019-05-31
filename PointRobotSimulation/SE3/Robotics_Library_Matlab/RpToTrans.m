function [ T ] = RpToTrans( R, p )

%Usage:
%Input - R - SO(3) and p R^3
%Output - Transformation Matrix T (SE(3))


epsilon = 10^(-5);  
T = [];
if(size(p,2) > 1)%if row convert to column
    p = p';
end
% check for Rp is valid rotation
if (numel(size(R)) ~= 2 || size(R,1) ~= size(R,2))
    disp('R is not a valid rotation matrix either because it has more than 2 dimensions or its not square');
elseif(size(R,1) ~= 3)
    disp('R is not 3 by 3');
elseif (CheckIfIdentity(R'*R) == false || (abs(det(R)) - 1) > epsilon) %check if R satisfies SO(3) conditions
    disp('R is not  valid rotation matrix because its not SO(3)');
elseif(size(p,2) > 1) % if a matrix
% check if p is 3,1 vector
    disp('p is not a vector');
elseif(size(p, 1) ~= 3)
    disp('p is not a 3d vector');
else
    %both R and p are valid at this point. 
    T(1:3, 1:3) = R;
    T(1:3, 4) = p;
    T(4,:) = [0,0,0,1];
end

