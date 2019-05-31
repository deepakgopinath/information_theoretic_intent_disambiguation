function [ q ] = AxisAng_Quat( r, varargin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Usage
% Input - r = exponential coordinates for rotation r = w*theta. Varargin is
% a flag used to change the output representation from [x,y,z,w] to [w,x,y,z]
% Output - Unit Quaternion representing the same rotations = [ qx, qy,
% qz, qw]

%Example
% Input - r = [pi/2, 0, 0]
% Output q = [0.707, 0.707, 0, 0] in [w,x,y,z]

q = [];
if(size(r,2) > 1) %Make it a column vector if a row vector
    r = r';
end
if(size(r,2) > 1) % if a matrix
    disp('r is not a vector');
elseif(size(r, 1) ~= 3)
    disp('r is not a 3d vector');
else
    [w,theta] = AxisAng3(r);
    qw = cos(theta/2);
    qv = sin(theta/2)*w;
    q = [qv;qw];
    if length(varargin) == 1
        q = circshift(q, 1);
    end
end
end

