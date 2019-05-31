function [ r ] = MatrixLog3( R )


%Usage
% Input - A valid rotation matrix SO(3)
% Output - exponential coordinates r = S*theta

epsilon = 10^(-5);
r= [];
if (numel(size(R)) ~= 2 || size(R,1) ~= size(R,2))
    disp('R is not a valid rotation matrix either because it has more than 2 dimensions or its not square');
elseif(size(R,1) ~= 3)
    disp('R is not 3 by 3');
elseif (CheckIfIdentity(R'*R) == false || (abs(det(R)) - 1) > epsilon) %check if R satisfies SO(3) conditions
    disp('R is not  valid rotation matrix because its not SO(3)');
else 
    %here this is a valid rotation matrix
    %case 1 - R is identity
    if(CheckIfIdentity(R) == true)
        theta = 0;
        r = [0,0,0]'; %when R is identity exponential coordinates is not defined. There vector of zero.
    elseif (trace(R) == -1)
        theta = pi;
        if((1 + R(3,3)) > 0) %a feasible solution happens when the term inside the square root is > 0
            omega = (1/sqrt(2*(1+R(3,3))))*[R(1,3);R(2,3);1+R(3,3)];
        elseif((1 + R(2,2)) > 0)
            omega = (1/sqrt(2*(1+R(2,2))))*[R(1,2);1+R(2,2);R(3,2)];
        elseif((1 + R(1,1)) > 0)
            omega = (1/sqrt(2*(1+R(1,1))))*[1+R(1,1);R(2,1);R(3,1)];
        end
        r = omega*theta;
    else
        theta =  acos((trace(R) - 1)/2);
        ssMatrix = (1/(2*sin(theta)))*(R - R'); %[w]
        r = so3ToVec(ssMatrix)*theta; %r = w*theta
    end
end

end

