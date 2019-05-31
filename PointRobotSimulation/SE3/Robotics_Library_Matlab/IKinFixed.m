function [ JAMatrix ] = IKinFixed(S, M, Tsd, theta0, epsw, epsv )

% Input - S - Screw Axes in Fixed Frame. 6 by n;
%         M - Initial Configuration when joint angle are all zero - SE(3) - 4 by 4
%         Tsd - Desired configuration in SE3
%         theta0 - initial guess for the joint angles
%         epsw - tolerance for rotation
%         epsv - tolerance for translation

% Output - JAMatrix - A list of joint angles corresponding to each iteration. numIter by numJoints

JAMatrix = [];
if(size(theta0, 2) > 1)
    theta0 = theta0';
end
epsilon = 10^-5;
if (numel(size(M)) ~= 2 || size(M,1) ~= size(M,2))
    disp('M is not a valid transformation matrix either because it has more than 2 dimensions or its not square');
elseif(size(M,1) ~= 4)
    disp('M is not 4 by 4');
elseif( sum(M(4,:) == [0,0,0,1]) ~= 4)
    disp('M is not a valid transformation matrix because last row is not [0,0,0,1]')
elseif(CheckIfIdentity(M(1:3,1:3)'*M(1:3,1:3)) == false || (abs(det(M(1:3,1:3))) - 1) > epsilon)
    disp('M is not a valid transformation matrix because the R component is not SO(3)');
elseif(size(S, 1) ~= 6)
    disp('Screw axes not 6 dimensional');
elseif(size(theta0, 2) > 1)
    disp('Theta is not a list of joint angles/translations');
elseif(size(S, 2) ~= length(theta0))
    disp('Number of joints mismatch inS and theta');
else
    
    thetai = theta0;
    maxiter = 100;
    i = 0;
    JAMatrix = [];
    
    
    Tsbthetai = FKinFixed(M, S, thetai); %SE3 at current theta
    Vb = MatrixLog6(TransInv(Tsbthetai)*Tsd); % Velocity vector in body frame
    Vs = Adjoint(Tsbthetai)*Vb; % Same velocity vector in fixed frame
    JAMatrix = [JAMatrix; thetai']; % append the joint angles to the JAMatrix
    while( (norm(Vs(1:3)) > epsw) || (norm(Vs(4:6)) >  epsv) ) % until stopping criterion is met
        thetaiplus1 = thetai + pinv(FixedJacobian(S, thetai))*Vs; % update theta
        i = i+1; % update i.
        if (i >= maxiter)
            disp('Reached max iteration limit');
            break;
        end
        thetai = thetaiplus1; %Reassign theta. 
        JAMatrix = [JAMatrix; thetai']; % Continue until convergence
        Tsbthetai = FKinFixed(M, S, thetai);
        Vb = MatrixLog6(TransInv(Tsbthetai)*Tsd);
        Vs = Adjoint(Tsbthetai)*Vb;
    end

end
end

