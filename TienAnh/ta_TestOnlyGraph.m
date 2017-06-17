close all
clear
clc
%  homedir = E:\Tien Anh\HOC TAP\HUMANOID\Introduction Humanoid Robotics
homedir = pwd;
matfiles_dir = sprintf('%s%sMatfiles',homedir,'\');
myfucn_dir = sprintf('%s%sTienAnh',homedir,'\');
addpath(matfiles_dir);
addpath(myfucn_dir);
load(sprintf('%s%sC_Ref.mat',matfiles_dir,'\'));
load(sprintf('%s%sFoot.mat',matfiles_dir,'\'));

FootTrajectory;
global uLINK G M
G = 9.8;  % g [m/s^2]
fig = 0;
SetupBipedRobot2;    % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)
M = TotalMass(1);
figure(2);
idx=1;

%%%%%% Initial condition %%%%%
uLINK(BODY).p = [0.0, 0.0, 0.6]';
uLINK(BODY).R = eye(3);
uLINK(BODY).v = [0, 0, 0]';
uLINK(BODY).w = [0, 0, 0]';
idx = 1;
% Right Foot
Rfoot.p = [Foot(idx,3), -0.1, Foot(idx,5)]';
Rfoot.R = RPY2R([0,Foot(idx,7),0]);  %  -pi/4 < q < pi/4
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';

% Left Foot
Lfoot.p = [Foot(idx,2), 0.1, Foot(idx,4)]';
Lfoot.R = RPY2R([0,Foot(idx,6),0]); %  -pi/4 < q < pi/4
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';

% InverseKinematics
InverseKinematicsAll(RLEG_J5, Rfoot);
InverseKinematicsAll(LLEG_J5, Lfoot);
ForwardVelocity(1);

% ta_Draw3DBall(0.02,Rfoot.p,'r');
% ta_Draw3DBall(0.02,Lfoot.p,'b');

com = calcCoM ;    % Position of the center of gravity
Zc  = com(3) ;     % Setting the height of the linear inverted pendulum
Tc  = sqrt(Zc/G);  % The time constant of the linear inverted pendulum
cx0 = com(1);
cy0 = com(2);
offset =[0.0025;-0.0340;-0.1218];
P1 = calcP(1);
L1 = calcL(1);
% ta_Draw3DBall(0.02,com,'g');
%%%%%% End Initial postion %%%%%

% Foot= [times LeftX RightX LeftZ RightZ LeftTheta RightTheta]
time = Foot(:,1);
tsize = length(time);
for idx=2:1    
%     idx = idx + 1;
    [px,vx] = LIPM(time(idx),Lfoot.p(1),cx0,Tc);
    [py,vy] = LIPM(time(idx),Lfoot.p(2),cy0,Tc);
%     uLINK(BODY).p = [px, py, 0.5]';
    temp = C_Ref(idx,:)'-offset;
    uLINK(BODY).p = temp;
    uLINK(BODY).v = [vx, vy, 0.0]';
    Rfoot.p = [Foot(idx,3), -0.1, Foot(idx,5)]';    
    Rfoot.R = RPY2R([0,Foot(idx,7),0]);  %  -pi/4 < q < pi/4
%     ta_Draw3DBall(0.02,Rfoot.p,'r');
    % InverseKinematicsAll(Rfoot,RLEG_J5);
    Lfoot.p = [Foot(idx,2), 0.1, Foot(idx,4)]';
    Lfoot.R = RPY2R([0,Foot(idx,6),0]); %  -pi/4 < q < pi/4
    ta_Draw3DBall(0.02,Lfoot.p,'b');
    % InverseKinematicsAll(Lfoot,LLEG_J5);
    qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
    qL2 = IK_leg(uLINK(BODY), 0.1,0.3,0.3,Lfoot);
    uLINK(BODY).p = ta_PositionBody(C_Ref(idx,:)');
%     ta_Draw3DBall(0.02,temp,'g');
%     uLINK(BODY).p = [0.1, 0.0, 0.5]';
    uLINK(BODY).R = eye(3);
    for n=0:5
    %     fprintf('%8s.q = %7.2f (deg)\n',uLINK(RLEG_J0+n).name,rad2deg(qR2(n+1)));
    %     fprintf('%8s.q = %7.2f (deg)\n',uLINK(LLEG_J0+n).name,rad2deg(qL2(n+1)));
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qL2(n+1);
    end
    ForwardKinematics(1);
    ForwardVelocity(1);    
    ShowRobot();
    
    pause(0.02);
end