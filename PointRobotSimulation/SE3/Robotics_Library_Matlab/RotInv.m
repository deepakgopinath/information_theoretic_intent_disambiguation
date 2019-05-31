function [ Rinv ] = RotInv( R )

%   usage - RotInv(R)
%   input (R) - A valid rotation matrix
%   output (Rinv) - The inverse of the valid rotation matrix. If not valid, error
%   message will be shown and an empty array will be returned



Rinv = [];
epsilon = 10^(-7); %Predefined error threshold value
%first check if the two dimensions are the same. R has to be square
if (numel(size(R)) ~= 2 || size(R,1) ~= size(R,2))
    disp('R is not a valid rotation matrix either because it has more than 2 dimensions or its not square');
elseif(size(R,1) ~= 3)
    disp('R is not 3 by 3');
elseif (CheckIfIdentity(R'*R) == false || (abs(det(R)) - 1) > epsilon) %check if R satisfies SO(3) conditions
    disp('R is not  valid rotation matrix because its not SO(3)');
else 
    Rinv = R'; %Inverse is the transpose for SO(3);
end
end

