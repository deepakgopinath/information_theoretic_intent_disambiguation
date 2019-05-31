function [ spaceJ ] = FixedJacobian( S, theta )

%Usage
%S - 6 by n matrix. 6 dimensions and n joints
%theta - n by 1 vector, containing all joint angles for the n joints

if(size(theta, 2) > 1)
    theta = theta';
end

spaceJ = [];
if(size(S, 1) ~= 6)
    disp('Screw axes not 6 dimensional');
elseif(size(theta, 2) > 1)
    disp('Theta is not a list of joint angles/translations');
elseif(size(S, 2) ~= length(theta))
    disp('Number of joint mismatch in S and theta');
else
    numJoints = size(S, 2);
    %now construct Jacobian column by column. 
    %Jacobian is going to be 6 by numJoints
    spaceJ = zeros(6, numJoints);
    spaceJ(:, 1) = S(:, 1); %initialize first column. 
    for i=2:numJoints
        prodT = eye(4,4); %reinitialize the accumulating product before each new column
        for j=1:i-1
            prodT = prodT*MatrixExp6(S(:,j)*theta(j));
        end
        spaceJ(:, i) = Adjoint(prodT)*S(:, i);
    end
end
end

