function [ q ] = RToQuat( R )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

aa = MatrixLog3( R );
q = AxisAng_Quat(aa);
end

