function [ JAMatrix ] = IKinBody( B, M, Tsd, theta0, epsw, epsv )

% Input - B - Screw Axes in Body Frame. 6 by n;
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
elseif(size(B, 1) ~= 6)
    disp('Screw axes not 6 dimensional');
elseif(size(theta0, 2) > 1)
    disp('Theta is not a list of joint angles/translations');
elseif(size(B, 2) ~= length(theta0))
    disp('Number of joints mismatch in B and theta');
else
    %checks on epsilons also needed. Add more if else
    %initialization
    thetai = theta0;
    maxiter = 1000;
    i = 0;
    JAMatrix = [];
    
    Tsbthetai = FKinBody(M, B, thetai);
    Vb =  MatrixLog6(TransInv(Tsbthetai)*Tsd); % Velocity vector in body frame
    JAMatrix = [JAMatrix; thetai'];
    while( (norm(Vb(1:3)) > epsw) || (norm(Vb(4:6)) >  epsv) )
        thetaiplus1 = thetai + pinv(BodyJacobian(B, thetai))*Vb; % column vectors
        i = i + 1;
        thetai = thetaiplus1;
        if (i >= maxiter)
            disp('Reached max iteration limit');
            break;
        end
        %thetai is already the updated value here.
        JAMatrix = [JAMatrix; thetai']; %append row vectors. 
        Tsbthetai = FKinBody(M, B, thetai);
        Vb =  MatrixLog6(TransInv(Tsbthetai)*Tsd);
        
    end
end
end

