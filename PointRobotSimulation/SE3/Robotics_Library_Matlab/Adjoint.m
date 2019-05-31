function [ AdT ] = Adjoint(T)

%usage:
%Input - A Valid SE(3) Matrix
%output - Adjoint of T, 6 by 6


AdT = [];
epsilon = 10^(-5);
if (numel(size(T)) ~= 2 || size(T,1) ~= size(T,2))
    disp('T is not a valid transformation matrix either because it has more than 2 dimensions or its not square');
elseif(size(T,1) ~= 4)
    disp('T is not 4 by 4');
elseif( sum(T(4,:) == [0,0,0,1]) ~= 4)
    disp('T is not a valid transformation matrix because last row is not [0,0,0,1]')
elseif(CheckIfIdentity(T(1:3,1:3)'*T(1:3,1:3)) == false || (abs(det(T(1:3,1:3))) - 1) > epsilon)
    disp('T is not a valid transformation matrix because the R component is not SO(3)');
else
    AdT = zeros(6,6);
    AdT(1:3, 1:3) = T(1:3, 1:3);
    AdT(4:6, 4:6) = T(1:3, 1:3);
    AdT(4:6, 1:3) = VecToso3(T(1:3, 4))*T(1:3, 1:3);
end

end

