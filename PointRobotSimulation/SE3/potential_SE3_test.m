clear all; clc; close all;
%%
load('test_data.mat');
rT = r;
gT = g;
% disp(gT); disp(rT);

xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];
th_range = [0, 2*pi];

l_axis = 0.1;
figure;
% trplot(rT, 'rgb', true, 'thick', 2.0, 'length', l_axis); hold on;
% trplot(gT, 'rgb', true, 'thick', 2.0, 'length', l_axis); hold on;
% scatter3(gT(1, 4), gT(2, 4), gT(3, 4), 230, 'm', 'filled'); grid on; hold on;
% scatter3(rT(1, 4), rT(2, 4), rT(3, 4), 230, 'k', 'filled'); grid on; hold on;
Twg = [0, 0, -1, 0.1;
      0, -1, 0, 0.2;
      -1, 0, 0, 0;
      0, 0, 0, 1];

Twr = [ 0, 1, 0, -0.1;
       0, 0, 1, 0.2;
       1, 0, 0, 0;
       0, 0, 0, 1];
trplot(Twg, 'rgb', true, 'thick', 2.0, 'length', l_axis); hold on;
trplot(Twr, 'rgb', true, 'thick', 2.0, 'length', l_axis); hold on;
scatter3(Twg(1, 4), Twg(2, 4), Twg(3, 4), 100, 'm', 'filled'); grid on; hold on;
scatter3(Twr(1, 4), Twr(2, 4), Twr(3, 4), 100, 'k', 'filled'); grid on; hold on;
offset = [-0.3, 0.3];
line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
axis([xrange+offset, yrange+offset, zrange+offset]);
axis square;
view([142,31]);


%% test homogenous matrix transformation
Tgr = Twg^(-1)*Twr;
Trg = Tgr^-1;
% disp(Twg); disp(Twr); disp(Tgr);
Tdiff = Twg*(Twr^-1);
[Rwr, pwr] = tr2rt(Twr);
[Rwg, pwg] = tr2rt(Twg);

Rdiff_w = Rwg*(Rwr^-1);
Rdiff_b = (Rwr^-1)*Rwg;

%%

Rwr = [ 0, 1, 0;
        -1, 0, 0;
        0 0 1];
Rwg = [ 0, 1, 0;
        0, 0, -1;
        -1, 0, 0];
    
%actual difference between 
Rdiff_w = [ 1, 0, 0;
            0, 0, -1;
            0, 1, 0];
