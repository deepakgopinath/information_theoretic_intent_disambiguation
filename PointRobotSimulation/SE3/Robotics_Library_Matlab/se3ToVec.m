function [ spatvel ] = se3ToVec(seMatrix)
%Usage:
%Input - A valid se(3) matrix
%Output - 6d spatial velocity vector. 6 by 1 vector

%Example:
%Input
%      0    -2     1    -2
%      2     0   -11    -4
%     -1    11     0     5
%      0     0     0     0  
%Output - [11,1,2,-2,-4,5]'

%First check if seMatrix is a valid matrix
spatvel = [];
if(numel(size(seMatrix)) ~= 2)
    disp('seMatrix is not 2 dimensional')
elseif(size(seMatrix, 1) ~= 4 || size(seMatrix, 2) ~= 4)
    disp('seMatrix is not 4 by 4');
elseif(sum(seMatrix(4,:) == [0,0,0,0]) ~= 4)
    disp('The last row of seMatrix is not [0,0,0,0]');
elseif(sum(diag(seMatrix(1:3,1:3)) == [0,0,0]') ~= 3)
    disp('The omega part of seMatrix is not skew symmetric because the diagonal elements are not zero');
elseif(seMatrix(1,2) ~= -seMatrix(2,1) || seMatrix(3,1) ~= -seMatrix(1,3) || seMatrix(2,3) ~= -seMatrix(3,2))
    disp('The omega part of seMatrix is not skew symmetric');    
else
    spatvel = zeros(6,1);
    spatvel(1:3) = so3ToVec(seMatrix(1:3,1:3));% extract omega from the R part
    spatvel(4:6) = seMatrix(1:3, 4);
end

end

