clear;
clc;

% Rz = MatrixExp3([0,0,pi/2]);
% Ry = MatrixExp3([0,pi/2,0]);
% 
% Rbody = Rz*Ry;
% Rworld = Ry*Rz;
% 
% aaz = MatrixLog3(Rz);
% aay = MatrixLog3(Ry);
% 
% qz = AxisAng_Quat(aaz);
% qy = AxisAng_Quat(aay);
% 
% 
% 
% qbody_1 = RToQuat(Rbody);
% qbody_2 = qmult(qz,qy);
% 
% qworld_1 = RToQuat(Rworld);
% qworld_2 = qmult(qy,qz);
% % qworld = qmult(qy,qz);
% 
% Rbfromqb = MatrixExp3([])


% Rsa = [1 0 0; 0 0 -1; 0 1 0];

% Rsb = [-1 0 0; 0 0 1; 0 1 0];
% 
% qsa = RToQuat(Rsa);

% qsb = RToQuat(Rsb);

%Destination stuff;
Rab = [-1 0 0; 0 1 0; 0 0 -1];
Rsa = [0 -1 0;0 0 -1;1 0 0];
Rsb = [0 0 -1; 0 -1 0; -1 0 0];

qsa = [0.438, -0.430, 0.546, 0.571]'; 
qab = RToQuat(Rab);