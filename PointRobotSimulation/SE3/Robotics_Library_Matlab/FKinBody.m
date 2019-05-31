function [ T ] = FKinBody( M, B, theta )

%Usage:
%Input:
%B - 6 by n matrix. 6 dimensions and n joints
%theta - n by 1 vector, containing all joint angles for the n joints
%M - is SE(3)


T = [];
epsilon = 10^(-5);
if(size(theta, 2) > 1)
    theta = theta';
end

if (numel(size(M)) ~= 2 || size(M,1) ~= size(M,2))
    disp('M is not a valid transformation matrix either because it has more than 2 dimensions or its not square');
elseif(size(M,1) ~= 4)
    disp('M is not 4 by 4');
elseif( sum(M(4,:) == [0,0,0,1]) ~= 4)
    disp('M is not a valid transformation matrix because last row is not [0,0,0,1]')
elseif(CheckIfIdentity(M(1:3,1:3)'*M(1:3,1:3)) == false || (abs(det(M(1:3,1:3))) - 1) > epsilon)
    disp('M is not a valid transformation matrix because the R component is not SO(3)');
elseif(size(B, 1) ~= 6)
    disp('Screw axes not 6 dimensional');
elseif(size(theta, 2) > 1)
    disp('Theta is not a list of joint angles/translations');
elseif(size(B, 2) ~= length(theta))
    disp('Number of joints mismatch in B and theta');
else
    T = M; % initialize T with M. 
    for i=1:length(theta)
        localB = B(:,i); % this will take care of the case in which the omega part of the screw axis is 0. 
        localTheta = theta(i);
        ssMatrix = VecToso3(localB(1:3));
        R = eye(3) + sin(localTheta)*(ssMatrix) + (1 - cos(localTheta))*(ssMatrix^2); %e^[w]theta
        localT = zeros(4,4);
        localT(1:3,1:3) = R;
        localT(4,:) = [0,0,0,1];
        localT(1:3, 4) = (eye(3)*localTheta + (1 - cos(localTheta))*ssMatrix + (localTheta - sin(localTheta))*(ssMatrix^2))*localB(4:6);
        T = T*localT; %recursive post multiplication. 
    end

end

