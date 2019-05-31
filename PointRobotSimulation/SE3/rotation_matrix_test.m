clear all; clc; close all;

%%
Rab = [0, -1, 0;
        1 0 0;
        0 0 1];
Rac = [0, -1, 0;
        0 0 -1;
        1 0 0];
Rbc = [0 0 -1;
       0 1 0;
       1 0 0];
%%
Rwa = [ 0 1 0;
        0 0 -1;
        -1 0 0];
pwa = [-0.2; -0.3; 0.2];
R = MatrixExp3([0, pi/2, 0]); %Rotation about y axis. 
Rwap = Rwa*R; %roatation of R with respect to body frame
Rwapp = R*Rwa; %rotation of R with respect to world frame

%%
Twa = [Rwa pwa;0 0 0 1];
Tab = [ 0 -1 0 0;
        -1 0 0 0;
        0 0 -1 0.16;
        0 0 0 1];
    
Twb = Twa*Tab;
%%
%%AFTER ROTATION OF A WRT TO BODY FRAME
Rwap = Rwa*R; 
%%AFTER TRANSLATION OF A WRT TO WORLD FRAME. 
pwap = [-0.3; -0.3; 0.2];
Twap =  [Rwap pwap;0 0 0 1];
Twbp = Twap*Tab;
