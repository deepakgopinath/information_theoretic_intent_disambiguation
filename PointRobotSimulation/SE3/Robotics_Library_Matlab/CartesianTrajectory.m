function [ SEList ] = CartesianTrajectory( X_s, X_end, T, N, tsMethod)

%Input - X_s - Initial configuration in SE3
%          X_end - Final configuration in SE3
%          T - Total time for the trajectory
%          N - Number of steps to be taken
%          tsMethod - 3 or 5. Flag for which time scaling method (cubic or quintic to be used)
% Output - SEList - 4 by 4 by N.  A List of N SE(3) Matrices representing the trajectory from X_s to X_end. 


SEList = [];
if(CheckIfValidSE3(X_s) == false || CheckIfValidSE3(X_end) == false)
    disp('Start or end transformation matrices are not valid')
elseif(tsMethod ~=3 && tsMethod ~= 5)
    disp('Please enter 3 for cubic, or 5 for Quintic Time Scaling')
elseif(N < 2)
    disp('Enter number of steps to be greater than or equal to 2')
elseif(T <= 0)
    disp('Invalid total Time. Has to be positive ');
else
    if(tsMethod == 3)
     timescalingmethod = @CubicTimeScaling;
        elseif(tsMethod == 5)
     timescalingmethod = @QuinticTimeScaling;
    end
    
    R_s = X_s(1:3,1:3); R_end = X_end(1:3,1:3);
    p_s = X_s(1:3,4); p_end = X_end(1:3,4);
    step = T/(N-1);
    t = (0:step:T)';
    s_t = timescalingmethod(T, t); %generate s(t), column vector, N
    logArg = MatrixLog3(R_s'*R_end);%Calculate Log once and for all
    SEList = zeros(4,4,N);
    
    for i=1:N
        SEList(1:3,4,i) = (1 - s_t(i))*p_s + s_t(i)*p_end; %Straight line for the translational
        SEList(1:3,1:3,i) = R_s*MatrixExp3(logArg*s_t(i));
        SEList(4,:,i) = [0,0,0,1]; %Make all the last rows of each of T matrix to be 0001
    end
end
end

