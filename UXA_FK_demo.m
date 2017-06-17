%%%  calculate_zmp.m
%%%  ZMP concrete example calculation and display of (ZMP) 
%    "3.4.1 ZMP derivation of"
close all
% clear all
clc
% open_uxa_serial;
% pause(2)
% uic_motion_cmd('pc_control');
% pause(10);

global uLINK G M Dtime
G = 9.8;  % g [m/s^2]
Dtime = 0.01;
doplot = 1;
SetupBipedRobot;   % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)

RJoints = zeros(1,6);
LJoints = zeros(1,6);

M = TotalMass(1);
%%%%%% Initial condition %%%%%
uLINK(BODY).p = [0.0, 0.0, 0.462]';
uLINK(BODY).R = eye(3);
uLINK(BODY).v = [0, 0, 0]';
uLINK(BODY).w = [0, 0, 0]';
uLINK(BODY).R = eye(3);
% uLINK(BODY).T = [0 -1 0 0;
%                      1 0 0 0;
%                      0 0 1 0.462;
%                      0 0 0 1];
% uLINK(BODY).T = [0 0 1 0;
%                  0 1 0 0;
%                 -1 0 0 0.462;
%                  0 0 0 1];                 
uLINK(BODY).T = [0 -1 0 0;
                 1 0 0 0;
                 0 0 1 0.462;
                 0 0 0 1];                 
                 
             
Rfoot.p = [0, -0.114/2, 0.07]';
% Rfoot.R = eye(3);
Rfoot.R = roty(90);
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';

Lfoot.p = [0, 0.114/2, 0.07]';
% Lfoot.R = eye(3);
Lfoot.R = roty(90);
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';
ForwardKinematics(1);

rotm2eul(uLINK(RLEG_J0+5).R)  % The default order for Euler angle rotations is 'ZYX'.
rotm2eul(uLINK(7).T(1:3,1:3)) % The default order for Euler angle rotations is 'ZYX'.
% Set all joint = 0 deg

clf
DrawAllJoints(1);
view(38,14)
axis equal
zlim([-0.07 1.3])
grid on
pause

uLINK(RLEG_J0).q   = 0*ToRad;
uLINK(RLEG_J0+1).q = 0*ToRad; %%
uLINK(RLEG_J0+2).q = 0*ToRad; 
uLINK(RLEG_J0+3).q = 0*ToRad; 
uLINK(RLEG_J0+4).q = 90*ToRad; 
uLINK(RLEG_J0+5).q = 0*ToRad; 

uLINK(LLEG_J0).q   = 0*ToRad;
uLINK(LLEG_J0+1).q = 0*ToRad;%% chieu!!!
uLINK(LLEG_J0+2).q = 0*ToRad; 
uLINK(LLEG_J0+3).q = 0*ToRad; 
uLINK(LLEG_J0+4).q = 0*ToRad; 
uLINK(LLEG_J0+5).q = 0*ToRad; 

ForwardKinematics(1);

% Get all joints angle in deg
% for idx = 1:13  
%     display([uLINK(idx).T(1:3,4) uLINK(idx).p]);
%     display([uLINK(idx).T(1:3,1:3)*inv(uLINK(idx).rot) uLINK(idx).R]);
%     rotm2eul(uLINK(idx).T(1:3,1:3))
%     
% %     display(uLINK(idx).p);
% end

% sam_bit = sendjointangle2uxa([RJoints LJoints]);

clf
DrawAllJoints(1);
view(38,14)
axis equal
zlim([-0.07 1.3])
grid on
