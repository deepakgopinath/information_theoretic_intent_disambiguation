clear all; clc; close all;

%%
Rwa = [ 1 0 0;
        0 0 -1;
        0 1 0];
Rwb = [ 1 0 0;
        0 -1 0;
        0 0 -1];
Rwc = [0, -1, 0;
        0,0,-1;
        1,0,0];
Rx = [  1 0 0;
        0 0 -1;
        0 1 0];
    
wbf = [0,0,1]';

delta_t = 0.1;
total_time = 2;
T = 0:delta_t:total_time;
d_ang = pi/2/length(T);
for i=1:length(T)+1
    disp(Rwa);
    Rwa = Rwa*MatrixExp3(d_ang*wbf); %body frame omega. For world frame omega, post multiply
    
    
%     w = d_ang*wbf;
%     a = norm(w);
%     uv = w/a;
%     u = sin(a/2.0)*uv;
%     z = cos(a/2.0);
%     q = [u;z];
%     R = q2dcm(q);
%     Rwa = Rwa*R;
    
end
