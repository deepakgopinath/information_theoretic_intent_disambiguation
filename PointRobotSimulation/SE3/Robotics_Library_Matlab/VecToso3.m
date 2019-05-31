function [ ssMatrix ] = VecToso3( inputVel )

%   usage - Vectoso3(inputVel)
%   input (inputVel) - A 3 vector representing angular velocity
%   output (ssMatrix) - A 3by3 skew symmetric matrix version of input
%   angular velocity. If the input is not valid, appropriate error message
%   will be shown

%   Example usage
%   input = [1,2,3];
%   output =    0    -3     2
%               3     0    -1
%-              2     1     0

ssMatrix =  [];
if(size(inputVel,2) > 1)
    inputVel = inputVel'; % make sure the input is in column form
end

if(size(inputVel, 2) > 1)
    disp('Input not a vector');
elseif(size(inputVel, 1) ~= 3)
    disp('Input not a 3-d vector');
else
    %Generate Skew Symmetric Matrix
    ssMatrix = [0, -inputVel(3), inputVel(2);
                inputVel(3), 0, -inputVel(1); 
                -inputVel(2), inputVel(1), 0];
end
end

