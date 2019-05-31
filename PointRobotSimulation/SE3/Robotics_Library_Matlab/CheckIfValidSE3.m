function [ isse3 ] = CheckIfValidSE3( M )

%Utility function to check whether a quantity M is a valid SE(3) matrix. 
%Input - M - scalar, matrix, vector....
%Output - True - if valid SE(3), False if not SE(3)

isse3 = false;
epsilon = 10^-5;
if (numel(size(M)) ~= 2 || size(M,1) ~= size(M,2))
    disp('M is not a valid transformation matrix either because it has more than 2 dimensions or its not square');
elseif(size(M,1) ~= 4)
    disp('M is not 4 by 4');
elseif( sum(M(4,:) == [0,0,0,1]) ~= 4)
    disp('M is not a valid transformation matrix because last row is not [0,0,0,1]')
elseif(CheckIfIdentity(M(1:3,1:3)'*M(1:3,1:3)) == false || (abs(det(M(1:3,1:3))) - 1) > epsilon)
    disp('M is not a valid transformation matrix because the R component is not SO(3)');
else
    disp('Valid SE3 input');
    isse3 = true;
end
end

