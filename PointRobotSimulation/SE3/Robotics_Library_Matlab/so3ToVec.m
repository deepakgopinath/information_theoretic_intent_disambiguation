function [ angVel ] = so3ToVec(ssMatrix)

%   angVel = so3ToVec(ssMatrix)
%   input - a skew symmetric matrix of so3 group (3 by 3)
%   output - retrieves the velocity ( 3 by 1)

%   Example usage
%   Input -      0    -3     2
%                3     0    -1
%               -2     1     0
%   Output - [1,2,3]'

% has to be 2-dimensional
angVel = [];
if(numel(size(ssMatrix)) ~= 2)
    disp('Skew symmetric matrix is not 2d');
elseif (size(ssMatrix, 1) ~= 3 || size(ssMatrix, 2) ~= 3) % if any one of the dimensions of the 2-d matrix is not 3 immediately abort
    disp('Skew symmetric matrix is not square and 3d')
elseif(sum(diag(ssMatrix) == [0,0,0]') ~= 3)
    disp('Non-Zero diagonal elements are present')
elseif(ssMatrix(1,2) ~= -ssMatrix(2,1) || ssMatrix(1,3) ~= -ssMatrix(3,1) || ssMatrix(2,3) ~= -ssMatrix(3,2))
    disp('Not a skew symmetric matrix');
else
    angVel = [ssMatrix(3,2);ssMatrix(1,3);ssMatrix(2,1)]; %extract the elements of the vector from the skew symmetric matrix and combine them to form the vector
end

