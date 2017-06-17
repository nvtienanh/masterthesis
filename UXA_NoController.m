%%%  calculate_zmp.m
%%%  ZMP concrete example calculation and display of (ZMP) 
%    "3.4.1 ZMP derivation of"
close all
clear all
clc

global uLINK G M Dtime
G = 9.8;  % g [m/s^2]
Dtime = 0.01;
doplot = 1;
SetupBipedRobot;   % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)
TF2A = [0 0 -1 0.065;
        0 1 0 0;
        1 0 0 0;
        0 0 0 1]; % Transform matrix from Foot coordinate to Ankle coordinate O_6
M = TotalMass(1);

%%%%%% Initial condition %%%%%
uLINK(BODY).vel = [0 0 0 0 0 0]';
uLINK(BODY).T = [0 -1 0 0;
                 1  0 0 0;
                 0  0 1 0.52;
                 0  0 0 1];  
ForwardKinematics(1); 

%%% Transformation matrix of Foot
% Right Foot
Rfoot.T = zeros(4,4);
Rfoot.T(1:3,1:3) = eye(3); % Rotation matrix
Rfoot.T(4,4) = 1;
%%% Transformation matrix of Foot
% Left Foot
Lfoot.T = zeros(4,4);
Lfoot.T(1:3,1:3) = eye(3); % Rotation matrix
Lfoot.T(4,4) = 1;

%%% Reference position in world coordination
Rfoot_p = [0, -0.114/2, 0]';
Rfoot_R = eye(3);
Rfoot.T(1:3,4) = Rfoot_p;
% Rfoot.T(1:3,1:3) = Rfoot_R;
% Rfoot.T(1:4,4) = [Rfoot_p ;1];

Lfoot_p = [0, 0.114/2, 0]';
Lfoot_R = eye(3);
Lfoot.T(1:3,4) = Lfoot_p;
% Lfoot.T(1:3,1:3) = Lfoot_R;
% Lfoot.T(1:4,4) = [Lfoot_p ;1];


tmp = Rfoot.T/TF2A;
Rfoot.p = tmp(1:3,4);
Rfoot.R = tmp(1:3,1:3);

Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';


tmp = Lfoot.T/TF2A;
Lfoot.p = tmp(1:3,4);
Lfoot.R = tmp(1:3,1:3);

Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';


[nr,rerr_norm] = ikine_sdls(RLEG_J5, Rfoot);
[nl,lerr_norm] = ikine_sdls(LLEG_J5, Lfoot);   
ForwardKinematics(1);
ForwardVelocity(1);

com = calcCoM ;    % Position of the center of gravity
Zc  = com(3);     % Setting the height of the linear inverted pendulum

dCoMHip = uLINK(BODY).T(1:3,4) - com;
P1 = calcP(1);
L1 = calcL(1);

walkstep = 0.05;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

% ZMPDesign      = [     0     walkstep walkstep walkstep     0;
%                   HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];              
[time,LAnkle,RAnkle,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef,ZMPout] = PatternGenerator(FootStepDesign,Zc);


times = time;
LeftAnkle = LAnkle;
RightAnkle = RAnkle;
dLeftAnkle = [[0 0 0];diff(LeftAnkle)]/Dtime;
dRightAnkle = [[0 0 0];diff(RightAnkle)]/Dtime;
tsize = length(times);
CoM_m = zeros(tsize,3); % CoM Multibody
ZMP_m = zeros(tsize,2); % % ZMP Multibody
ZMP_mInit = com(1:2);
LJoints = zeros(tsize,6);
RJoints = zeros(tsize,6);
joint_vel_1 = zeros(tsize,6);
joint_vel_2 = zeros(tsize,6);

foot_vel_1 = zeros(tsize,3);
foot_vel_2 = zeros(tsize,3);


figure
zmpz = 0.0;
vd = VideoWriter('BipedWalk.avi');
open(vd);

for k = 1:tsize     
    
    COM = CoM(k,:)';

    uLINK(BODY).T(1:3,4) = dCoMHip+COM;
    uLINK(BODY).v = dCoM(k,:)';
    
    Rfoot_p = RightAnkle(k,:)';
    Rfoot.T(1:3,1:3) = Rfoot_R;
    Lfoot_p = LeftAnkle(k,:)'; 
    Lfoot.T(1:3,1:3) = Lfoot_R;
   
    Rfoot.T(1:4,4) = [Rfoot_p ;1];
    Lfoot.T(1:4,4) = [Lfoot_p ;1];
    
    tmp = Rfoot.T/TF2A;
    Rfoot.p = tmp(1:3,4);
    
    tmp = Lfoot.T/TF2A;
    Lfoot.p = tmp(1:3,4);

    Lfoot.v = dLAnkle(k,:)';
    Rfoot.v = dRAnkle(k,:)';
    
    Lfoot.v1 = dLAnkle(k,:)';%
    Rfoot.v1 = dRAnkle(k,:)';%
    
    [nr,rerr_norm] = ikine_sdls(RLEG_J5, Rfoot);
    [nl,lerr_norm] = ikine_sdls(LLEG_J5, Lfoot);        
        
    for n=0:5        
        qR2(n+1) = uLINK(RLEG_J0+n).q;        
        qL2(n+1) = uLINK(LLEG_J0+n).q;
    end
    
    ForwardKinematics(1);
    ForwardVelocity(1); 
    
    LJoints(k,:) = qL2';
    RJoints(k,:) = qR2';
    %%% Calculation of ZMP
    com = calcCoM;  % Position of the center of gravity 
    
    %% Kalmafilter
    % ZMPx
    
    LinearMomentum = calcP(1);   % Translational momentum
    AngularMomentum = calcL(1);   % Angular momentum around the origin
    
    dP = (LinearMomentum-P1)/Dtime;
    dL = (AngularMomentum-L1)/Dtime;
    [zmpx,zmpy] = calcZMP(com,dP,dL,zmpz);

    P1 = LinearMomentum;
    L1 = AngularMomentum;
   
    CoM_m(k,:) = com';
    ZMP_m(k,:) = [zmpx, zmpy];
    dCoMHip = uLINK(BODY).p - com;
    hold off
    newplot
    DrawAllJoints(1);
    h(1) = DrawMarker([zmpx,zmpy,zmpz]','r');
    h(2) = DrawMarker([com(1),com(2),0]','b');
    legend(h,'IZMP','CoM');
    axis equal
    set(gca,...         
        'CameraPositionMode','auto',...
        'CameraPosition',[2,2,2],...
        'CameraViewAngleMode','auto',...
        'CameraViewAngle',20,...
        'Projection','orthographic',...
        'XLimMode','manual',...
        'XLim',[-0.3+com(1) 0.3+com(1)],...
        'YLimMode','manual',...
        'YLim',[-0.3+com(2) 0.3+com(2)],...
        'ZLimMode','manual',...
        'ZLim',[0 1])
    grid on
    text(0.5, -0.1, 1.1, ['time=',num2str(times(k),'%5.3f')])
    drawnow;
    frame = getframe;
    writeVideo(vd,frame);
end        
close(vd);
%% PLot useful graph
LJoints = rad2deg(LJoints);
RJoints = rad2deg(RJoints);
if doplot    
    figure
    subplot(2,1,1)
    hold on
    plot(times,CoM_m(:,1),'-r');
    plot(times,ZMPRef(:,1),'--g');
    plot(times,ZMPout(:,1),'--r');
    plot(times,ZMP_m(:,1),'-b');    
    title('ZMP and CoM in X direction');
    grid on
    subplot(2,1,2)
    hold on
    plot(times,CoM_m(:,2),'-r');
    plot(times,ZMPRef(:,2),'--g');
    plot(times,ZMPout(:,2),'--r');
    plot(times,ZMP_m(:,2),'-b');   
    title('ZMP and CoM in Y direction');
    grid on
    
    figure    
    hold on        
    plot(CoM_m(:,1),CoM_m(:,2),'-b');  
    plot(ZMP_m(:,1),ZMP_m(:,2),'-r'); 
    plot(ZMPRef(:,1),ZMPRef(:,2),'--r');
    plot(ZMPout(:,1),ZMPout(:,2),'--r');
    legend('CoM','ZMP','ZMP Ref','ZMPout');
    title('CoM and ZMP');   
    grid on
    
    figure   
    plot(times,LJoints);    
    grid on
    title('Left Leg Joints (deg)');
    legend('q_1','q_2','q_3','q_4','q_5','q_6');
    
    figure    
    hold on
    plot(times,RJoints);    
    grid on
    title('Right Leg Joints (deg)');
    legend('q_1','q_2','q_3','q_4','q_5','q_6');        
    
  
   
end
% LJoints = deg2rad(LJoints);
% RJoints = deg2rad(RJoints);
% RJoints(:,2) = -RJoints(:,2);% RAD
% LJoints(:,2) = -LJoints(:,2);% RAD
save 'SimData2UXA/RJoints.mat' RJoints % unit is Degree
save 'SimData2UXA/LJoints.mat' LJoints
save 'SimData2UXA/times.mat' times
save ZMP_m.mat ZMP_m

% figure
% subplot(3,1,1)
% hold on
% plot(times,foot_vel_1(:,1),'-r')
% plot(times,foot_vel_2(:,1),'--b')
% grid on
% subplot(3,1,2)
% hold on
% plot(times,foot_vel_1(:,2),'-r')
% plot(times,foot_vel_2(:,2),'--b')
% grid on
% subplot(3,1,3)
% hold on
% plot(times,foot_vel_1(:,3),'-r')
% plot(times,foot_vel_2(:,3),'--b')
% grid on

    