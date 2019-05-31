function [ SEList ] = ScrewTrajectory(X_s, X_end, T, N, tsMethod)

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
    if(tsMethod == 3) % Assign time scaling function handle
     timescalingmethod = @CubicTimeScaling;
        elseif(tsMethod == 5)
     timescalingmethod = @QuinticTimeScaling;
    end
    
    step = T/(N-1);  % define step size
    t = (0:step:T)'; %list of time steps
    s_t = timescalingmethod(T, t); %generate s(t), column vector, N
    SEList = zeros(4,4,N); % List of SE(3) matrices
    logArg = MatrixLog6(TransInv(X_s)*X_end); % 6 by 1. Calculate this once and for all. Saves computation
    for i=1:N
        SEList(:,:,i) = X_s*MatrixExp6(logArg*s_t(i)); % s_t(i) is a scalar. 
    end
    
end


end

