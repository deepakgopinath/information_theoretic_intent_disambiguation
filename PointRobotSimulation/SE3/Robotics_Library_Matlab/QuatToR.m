function [ R ] = QuatToR( q )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
r = QuatToAxisAng(q);
R = MatrixExp3(r);

end

