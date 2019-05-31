function [ seMatrix ] = VecTose3( spatvel ) % this is a full spatial velocity

%Usage
%Input - 6d spatial velocity
%Output -4 by 4 se(3) matrix. 

%Example
%Input = [11,1,2,-2,-4,5]'
%Output 
%      0    -2     1    -2
%      2     0   -11    -4
%     -1    11     0     5
%      0     0     0     0  

seMatrix = [];
if(size(spatvel, 2) > 1) %if row make it column
    spatvel= spatvel';
end

if(size(spatvel,2) > 1)
    disp('Input spatial velocity is not a vector');
elseif(size(spatvel,1) ~= 6)
    disp('Input spatial velocity is not 6-dimensional');
else
    seMatrix = zeros(4,4);
    seMatrix(1:3,1:3) = VecToso3(spatvel(1:3)); % the rotation part
    seMatrix(1:3, 4) = spatvel(4:6); %translation part
end

end

