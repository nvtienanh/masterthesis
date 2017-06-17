%%%  calculate_zmp.m
%%%  ZMP concrete example calculation and display of (ZMP) 
%    "3.4.1 ZMP derivation of"
close all
clear
clc
addpath('PatternGeneration')

global uLINK G M Dtime
G = 9.8;  % g [m/s^2]
Dtime = 0.01;
doplot = 1;
SetupBipedRobot2;   % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)

M = TotalMass(1);

%%%%%% Initial condition %%%%%
uLINK(BODY).p = [0.0, 0.0, 0.5]';
uLINK(BODY).R = eye(3);
uLINK(BODY).v = [0, 0, 0]';
uLINK(BODY).w = [0, 0, 0]';

Rfoot.p = [0, -0.114/2, 0.1]';
Rfoot.R = eye(3);
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';

Lfoot.p = [0, 0.114/2, 0.07]';
Lfoot.R = eye(3);
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';

% InverseKinematicsAll(RLEG_J5, Rfoot);
% InverseKinematicsAll(LLEG_J5, Lfoot);
qR2 = IK_UXAleg(uLINK(BODY),-0.114/2,0.21,0.21,Rfoot);
qL2 = IK_UXAleg(uLINK(BODY), 0.114/2,0.21,0.21,Lfoot);
for n=0:5 
    uLINK(RLEG_J0+n).q = qR2(n+1);        
    uLINK(LLEG_J0+n).q = qL2(n+1);         
end
ForwardKinematics(1);
com = calcCoM;     % Position of the center of gravity
Zc  = com(3);      % Setting the height of the linear inverted pendulum


clf
DrawAllJoints(1);
view(0,0)
axis equal
%     zlim([0.1 1.3])
grid on

%--------------------------------------------------------------------------
%

% for k = 1:tsize
%     %% PREVIEW CONTROL   
%     
%     COM = [CoM(k,1);CoM(k,2);CoM(k,3)];
%     uLINK(BODY).p = dCoMHip+COM;
%     uLINK(BODY).v = dCoM(k,:)';
% %     uLINK(BODY).v = [x(2) y(2) 0]';
%     Rfoot.p = RightAnkle(k,:)'; 
%     Lfoot.p = LeftAnkle(k,:)';
%     
%     qR2 = IK_leg(uLINK(BODY),-0.114/2,0.21,0.21,Rfoot);
%     qL2 = IK_leg(uLINK(BODY), 0.114/2,0.21,0.21,Lfoot);
%     LJoints(k,:) = qL2';
%     RJoints(k,:) = qR2';
%     for n=0:5  
%         uLINK(LLEG_J0+n).dq = (qL2(n+1)-uLINK(LLEG_J0+n).q)/Dtime;
%         uLINK(RLEG_J0+n).dq = (qR2(n+1)-uLINK(RLEG_J0+n).q)/Dtime;
%         uLINK(RLEG_J0+n).q = qR2(n+1);        
%         uLINK(LLEG_J0+n).q = qL2(n+1);         
%     end
%     ForwardKinematics(1);
%     ForwardVelocity(1);    
%     
%     %%% Calculation of ZMP
%     com = calcCoM;  % Position of the center of gravity
%     
%     com(1) = com(1); % Add noisy
%     com(2) = com(2); % Add noisy
%     CoM_nosy(k,:) = com';
%     
%     LinearMomentum = calcP(1);   % Translational momentum
%     AngularMomentum = calcL(1);   % Angular momentum around the origin
%     
%     dP = (LinearMomentum-P1)/Dtime;
%     dL = (AngularMomentum-L1)/Dtime;
%     [zmpx,zmpy] = calcZMP(com,dP,dL,zmpz);
%     uLINK(BODY).w
%     %End Kalmafilter
%     P1 = LinearMomentum;
%     L1 = AngularMomentum;
%    
%     CoM_m(k,:) = com';
%     ZMP_m(k,:) = [zmpx, zmpy];
%     dCoMHip = uLINK(BODY).p - com;
%     hold off
%     newplot
%     DrawAllJoints(1);
%     h(1) = DrawMarker([zmpx,zmpy,zmpz]','r');
%     h(2) = DrawMarker([com(1),com(2),0]','b');
%     legend(h,'IZMP','CoM');
%     axis equal
%     set(gca,...
%         'CameraPositionMode','manual',...
%         'CameraPosition',[4,4,2],...
%         'CameraViewAngleMode','manual',....
%         'CameraViewAngle',20,...
%         'Projection','perspective',... 
%         'XLimMode','manual',...
%         'XLim',[-0.4+com(1) 0.4+com(1)],...
%         'YLimMode','manual',...
%         'YLim',[-0.4+com(2) 0.4+com(2)],...
%         'ZLimMode','manual',...
%         'ZLim',[0 1])
%     grid on
%     text(0.5, -0.1, 1.4, ['time=',num2str(times(k),'%5.3f')])
%     drawnow;
%     frame = getframe;
%     writeVideo(vd,frame);
% end        

