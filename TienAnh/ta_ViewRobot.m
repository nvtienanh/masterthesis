%%%  calculate_zmp.m
%%%  ZMP concrete example calculation and display of (ZMP) 
%    "3.4.1 ZMP derivation of"
close all
clear
clc

homedir = pwd;
matfiles_dir = sprintf('%s%sMatfiles',homedir,'\');
myfucn_dir = sprintf('%s%sTienAnh',homedir,'\');
addpath(matfiles_dir);
addpath(myfucn_dir);
load(sprintf('%s%sC_Ref.mat',matfiles_dir,'\'));
load(sprintf('%s%sFoot.mat',matfiles_dir,'\'));

global uLINK G M
G = 9.8;  % g [m/s^2]

SetupBipedRobot2;   % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)

M = TotalMass(1);

%%%%%% Initial condition %%%%%
uLINK(BODY).p = [0.0, 0.0, 0.5]';
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

InverseKinematicsAll(RLEG_J5, Rfoot);
InverseKinematicsAll(LLEG_J5, Lfoot);
ForwardVelocity(1);

com = calcCoM;     % Position of the center of gravity
Zc  = com(3);      % Setting the height of the linear inverted pendulum
Tc  = sqrt(Zc/G);  % The time constant of the linear inverted pendulum
cx0 = com(1);
cy0 = com(2);
offset = com - uLINK(BODY).p;
P1 = calcP(1);
L1 = calcL(1);


global Dtime 

% Dtime = 0.01;
% EndTime = 0.5;
% time = 0:Dtime:EndTime;
% tsize = length(time);

% Foot= [times LeftX RightX LeftZ RightZ LeftTheta RightTheta]
time = Foot(:,1);
tsize = length(time);

com_m = zeros(tsize,3);
zmp_m = zeros(tsize,2);

figure
k = 1;
zmpz = 0.0;
for k = 1:tsize
    [px,vx] = LIPM(time(k),Lfoot.p(1),cx0,Tc);
    [py,vy] = LIPM(time(k),Lfoot.p(2),cy0,Tc);
    temp = C_Ref(k,:)'-offset;
    uLINK(BODY).p = temp;
%     uLINK(BODY).p = [px, py, 0.5]';
    uLINK(BODY).v = [vx, vy, 0.0]';
    
    InverseKinematicsAll(LLEG_J5, Lfoot); % To maintain the support legs 
%                                           on the ground
    InverseKinematicsAll(RLEG_J5, Rfoot);
    ForwardVelocity(1);    
    
    %%% Calculation of ZMP
    com = calcCoM;  % Position of the center of gravity
    P = calcP(1);   % Translational momentum
    L = calcL(1);   % Angular momentum around the origin
    
    dP = (P-P1)/Dtime;
    dL = (L-L1)/Dtime;
    [zmpx,zmpy] = calcZMP(com,dP,dL,zmpz);
    P1 = P;
    L1 = L;
   
    com_m(k,:) = com';
    zmp_m(k,:) = [zmpx, zmpy];
    
    hold off
    newplot
    DrawAllJoints(1);
    h(1) = DrawMarker([zmpx,zmpy,zmpz]','r');
    h(2) = DrawMarker([com(1),com(2),0]','b');
    legend(h,'IZMP','CoM');
    axis equal
    set(gca,...
        'CameraPositionMode','manual',...
        'CameraPosition',[4,4,1],...
        'CameraViewAngleMode','manual',....
        'CameraViewAngle',15,...
        'Projection','perspective',... 
        'XLimMode','manual',...
        'XLim',[-0.5+com(1) 0.5+com(1)],...
        'YLimMode','manual',...
        'YLim',[-0.5+com(2) 0.5+com(2)],...
        'ZLimMode','manual',...
        'ZLim',[-0.4 1.5])
    grid on
    text(0.5, -0.4, 1.4, ['time=',num2str(time(k),'%5.3f')])
    drawnow;
end        
figure(2)
plot(time,[com_m(:,1),zmp_m(:,1)]);
legend('IZMP','CoM');