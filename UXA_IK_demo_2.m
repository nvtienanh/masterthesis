close all
clear all
clc
addpath('myfunctions');
global uLINK       % ??????????????????????????

SetupBipedRobot;   % ?2.19??2.20?2??????????????
TF2A = [0 0 -1 0.065;
        0 1 0 0;
        1 0 0 0;
        0 0 0 1]; % Transform matrix from Foot coordinate to Ankle coordinate O_6
%%%%%%%%%%% ?????????%%%%%%%%%%%%
uLINK(RLEG_J2).q = 0.0*ToRad;
uLINK(RLEG_J3).q = 0.0*ToRad;
uLINK(RLEG_J4).q = 90.0*ToRad;
uLINK(RLEG_J5).q = 0.0*ToRad;

uLINK(LLEG_J2).q = 0.0*ToRad;
uLINK(LLEG_J3).q = 0.0*ToRad;
uLINK(LLEG_J4).q = 0.0*ToRad;

uLINK(BODY).T = [0 -1 0 0;
                 1  0 0 0;
                 0  0 1 0.527;
                 0  0 0 1];             
             
ForwardKinematics(1); 
%%%%%%%%%%% ????????????? %%%%%%%%%%%%
figure
DrawAllJoints(1);
view(10,20)
axis equal
zlim([-0.12 1.1]);
ylim([-0.15 .15]);
xlim([-0.1 .2]);
grid on
pause
 

% REFERENCE POSTURE OF FOOT (IN WORLD COORDINATION)
Rfoot_p = [0.0, -0.057, 0.1]';
Rfoot_R = eye(3);  %  -pi/4 < q < pi/4
% Rfoot.v = [0 0 0]';
% Rfoot.w = [0 0 0]';

Rfoot.T = zeros(4,4);
Rfoot.T(1:3,1:3) = Rfoot_R;
Rfoot.T(1:4,4) = [Rfoot_p ;1];

tmp = Rfoot.T/TF2A;
Rfoot.p = tmp(1:3,4);
Rfoot.R = tmp(1:3,1:3);

[nr,rerr_norm] = ikine_sdls(RLEG_J5, Rfoot);
% [nl,lerr_norm] = ikine_sdls(LLEG_J5, Lfoot);   

ForwardKinematics(1);
ForwardVelocity(1); 

clf
DrawAllJoints(1);
view(10,20)
axis equal
zlim([-0.12 1.1]);
ylim([-0.15 .15]);
xlim([-0.1 .2]);
grid on