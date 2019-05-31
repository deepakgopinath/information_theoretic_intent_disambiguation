function [ traj ] = JointTrajectory( theta_s, theta_end, T, N, tsMethod )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%Input - theta_s - Initial configuration in joint space
%          theta_end - Final configuration in joint space.
%          T - Total time for the trajectory
%          N - Number of steps to be taken
%          tsMethod - 3 or 5. Flag for which time scaling method (cubic or quintic to be used)

% Output - traj - N (number of time steps) by num_Joints. List of vectors indicating
% joint angles as a function of time

if(size(theta_s, 2) > 1)
    theta_s = theta_s';
end
if(size(theta_end, 2) > 1)
    theta_end = theta_end';
end

traj = [];

if(size(theta_s, 2) > 1 || size(theta_end, 2)> 1)
    disp('List of start or end joint angles not a vector')
elseif(tsMethod ~=3 && tsMethod ~= 5)
    disp('Please enter 3 for cubic, or 5 for Quintic Time Scaling')
elseif(N < 2)
    disp('Enter number of steps to be greater than or equal to 2')
elseif(T <= 0)
    disp('Invalid total Time. Has to be positive ');
else
   if(tsMethod == 3) %Generate time scaling function handle. 
     timescalingmethod = @CubicTimeScaling;
   elseif(tsMethod == 5)
     timescalingmethod = @QuinticTimeScaling;
   end
   num_joints = size(theta_s, 1);
   step = T/(N-1);
   t = (0:step:T);
   s_t = timescalingmethod(T, t); %generate s(t) % column
   
   s = repmat(s_t', num_joints, 1);
   oneminuss = 1-s;
   ts = repmat(theta_s, 1, length(t));
   te = repmat(theta_end, 1, length(t));
   traj =  (oneminuss.*ts + s.*te)'; %straight line path - vectorized. 
end

end


