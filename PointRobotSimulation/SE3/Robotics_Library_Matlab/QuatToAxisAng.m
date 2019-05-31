function [ r ] = QuatToAxisAng(q, varargin)
%UNTITLED4 Summary of this function goes here
%   Input - Quaternion: Default- [qx,qy,qz,qw], varargin flag is used to denote
%   that the quaternion is of form [qw,qx, qy,qz]
% output - r = omega*theta. The Axis Angle form. 

r = [];
if size(q, 2) > 1 %make q a column vector
    q = q';
end
if length(varargin) > 1 %if it x,y,z,w
    q = circshift(q, 1);
end

theta = 2*acos(q(4)); %get theta from scalar part
w = q(1:3)/sqrt(1 - q(4)^2); %deno is sin(theta/2);
r = w*theta;

end

