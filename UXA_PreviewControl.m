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
%% PREVIEW CONTROL INITIAL
preview_time=1;          % Preview Width
% calc_time=times(end);    % Time for Walk Pattern
NumPreview = length(0:Dtime:preview_time);
center_z=com(3);       %Position of Center Of Gravity(z)
A = [1 Dtime Dtime^2/2;
     0 1  Dtime;
     0 0  1];
B = [Dtime^3/6 Dtime^2/2 Dtime]';
C = [1 0 -com(3)/G];
D=[Dtime;1;0];              %Disturbance Matrix
%Error System
ZERO=[0;0;0];
A_hat=[1 -C*A;ZERO A];% phi=[1 -C_d*A_d;ZERO A_d];
B_hat=[-C*B;B];% G=[-C_d*B_d;B_d];
C_hat=[1;ZERO];% GR=[1;ZERO];
D_hat=[-C*D;D];     % Use the disturbance preview control (not used to ;
                    % walking pattern generation)

Q=zeros(4,4);   %Zero matrix
Q(1,1)=1e+8;      %Weighting factor (the ZMP is closer to the ideal system is 1e + 8)
R=1;          %Weight coefficient
 

%Riccati Equation
[~,P]=dlqr(A_hat,B_hat,Q,R);
K = -(R+B_hat'*P*B_hat)^(-1)*B_hat'*P*A_hat;
f_i = (eye(4,4)-B_hat*(R+B_hat'*P*B_hat)^(-1)*B_hat'*P)*A_hat;

AddZMPPreview = repmat(ZMPRef(end,:),length(0:Dtime:preview_time),1);
ZMP_IP = [ZMPRef;AddZMPPreview];
%%------------------------------------------------------------------

figure
zmpz = 0.0;
vd = VideoWriter('BipedWalk.avi');
open(vd);
%% Init ZMP = CoM
zmpx = com(1);
zmpy = com(2);

x = [com(1);0;0];
y = [com(2);0;0];
xp = x;
yp = y;
%Initialized Increment Parameter
ux = 0;
uy = 0;
tmpF = zeros(length(times),1);

for k = 1:tsize
    %% PREVIEW CONTROL     
    ex = ZMP_IP(k,1) - zmpx;  %Error between the target ZMP(x)
    ey = ZMP_IP(k,2) - zmpy;  %Error between the target ZMP(y)
    X = [ex ; x - xp];
    Y = [ey ; y - yp];
    xp = x;
    yp = y; 
    % end init
    dux = K * X;    %State Feedback
    j = 0;
    gj = 0;
    for idxsup = k : Dtime : (k + preview_time) ;
        j = j + 1;
        if (ZMP_IP(k+j,1) - ZMP_IP(k+j-1,1)) ~= 0            
            f  = -(R+B_hat'*P*B_hat)^(-1)*B_hat'*(f_i')^(j-1)*P*C_hat ;       %ZMP feedforward term
            gj = gj+f;
            dux = dux + f*(ZMP_IP(k+j,1) - ZMP_IP(k+j-1,1));  
        end
    end
    ux = ux + dux;      %Control input
    tmpF(k) = gj;
    duy = K * Y;    %State Feedback
    j = 0;
    for idxsup = k : Dtime : (k + preview_time)
        j = j + 1;
        if (ZMP_IP(k+j,2) - ZMP_IP(k+j-1,2)) ~= 0
            f  = -(R+B_hat'*P*B_hat)^(-1)*B_hat'*(f_i')^(j-1)*P*C_hat;         %ZMP feedforward term
            duy = duy + f * (ZMP_IP(k+j,2) - ZMP_IP(k+j-1,2));
        end
    end
    uy = uy + duy;      %Control input

    dx=0.00;
    dy=0.00;
    x = A*x + B*ux + D*dx*Dtime;     %COG Trajectory(x)
    y = A*y + B*uy + D*dy*Dtime;     %COG Trajectory(y)    
    %%--------------------------------------------------------------------   
    
    COM = [x(1);y(1);CoM(k,3)];

    uLINK(BODY).T(1:3,4) = dCoMHip+COM;
    uLINK(BODY).v = [x(2) y(2) 0]';

    
    Rfoot_p = RightAnkle(k,:)';
    Rfoot.T(1:3,1:3) = Rfoot_R;
    Lfoot_p = LeftAnkle(k,:)'; 
    Lfoot.T(1:3,1:3) = Lfoot_R;
   
    Rfoot.T(1:4,4) = [Rfoot_p ;1];
    Lfoot.T(1:4,4) = [Lfoot_p ;1];
    
    tmp = Rfoot.T/TF2A;
    Rfoot.p = tmp(1:3,4);
%     Rfoot.R = tmp(1:3,1:3);
    
    tmp = Lfoot.T/TF2A;
    Lfoot.p = tmp(1:3,4);
%     Lfoot.R = tmp(1:3,1:3);

    Lfoot.v = dLAnkle(k,:)';
    Rfoot.v = dRAnkle(k,:)';
    
    Lfoot.v1 = dLAnkle(k,:)';%
    Rfoot.v1 = dRAnkle(k,:)';%
    
%     [~,rerr_norm] = InverseKinematicsAll(RLEG_J5, Rfoot);
%     [~,lerr_norm] = InverseKinematicsAll(LLEG_J5, Lfoot);
    [nr,rerr_norm] = ikine_sdls(RLEG_J5, Rfoot);
    [nl,lerr_norm] = ikine_sdls(LLEG_J5, Lfoot);        
        
    for n=0:5        
        qR2(n+1) = uLINK(RLEG_J0+n).q;        
        qL2(n+1) = uLINK(LLEG_J0+n).q;
    end
    
    ForwardKinematics(1);
    ForwardVelocity(1); 
    
%     foot_vel_1(k,:) = uLINK(RLEG_J4).vel(1:3)';
%     foot_vel_2(k,:) = uLINK(RLEG_J4).v';
%     display(uLINK(RLEG_J4).vel(1:3))
%     display(uLINK(RLEG_J4).v)
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
    
    figure
    plot(times(1:NumPreview),tmpF(NumPreview:-1:1));
    grid on
    title('Preview Gain');    
   
end
% LJoints = deg2rad(LJoints);
% RJoints = deg2rad(RJoints);
% RJoints(:,2) = -RJoints(:,2);% RAD
% LJoints(:,2) = -LJoints(:,2);% RAD
save 'SimData2UXA/RJoints.mat' RJoints % unit is Degree
save 'SimData2UXA/LJoints.mat' LJoints
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

    