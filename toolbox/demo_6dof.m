%%
clc;
clear all;
close all;

%% Graphic
g = ncgr_graphic();

%% Scara robot DH-Parameters
global N_DOFS;
N_DOFS = 6;

theta = [0 0 0 0 0 0];
alpha = [-pi/2 pi/2 0 0 pi/2 0];
offset = [pi/2 pi/2 0 0 0 0];
a = [0 0 0.21 0.21 0 0.065];
d = [-0.042 0 0 0 0 0];
type = ['r','r','r','r','r','r'];
base = [0; 0; 0];

lb = [-inf; -inf; -inf; -inf; -inf; -inf];
ub = [inf; inf; inf; inf; inf; inf];

scara = cgr_create(theta, d, a, alpha, offset, type, base, ub, lb);
scara = cgr_self_update(scara, [0; 0; -pi/6; pi/3; 0; 0]);
g = ncgr_plot(g, scara);

% pause(1);
% scara = cgr_self_update(scara, [0; 0; 0; -0.2]);
% g = ncgr_plot(g, scara);

% pause(1);
% scara = cgr_self_update(scara, [0; 0; 0; 0]);
% g = ncgr_plot(g, scara);

% Demo inverese kinematics
[q, k, err]= cgr_ikine1(scara, [0.0; 0.0; -0.4], 0.01, 100);
scara = cgr_self_update(scara, q);
g = ncgr_plot(g, scara);
pause(0.1);