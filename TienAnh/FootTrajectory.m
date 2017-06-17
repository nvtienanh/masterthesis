% FootTrajectory.m
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
close all
clear
clc
% addpath ../
global Tc Td To qb qf qs qe Lao Hao ltr lsh lth lan Ds lab laf xed xsd
global G Dtime
Tc = 1;
Td = 0.2*Tc;
To = 0.45*Tc;
qb = -0.2;
qf = 0.2;
% level ground
qs = 0;
qe = 0;

Lao = 0.13;
Hao = 0.08;
ltr = 0.06;
lth = 0.04;
lsh = 0.04;
lan = 0.01;
lab = 0.01;
laf = 0.013;
Ds = 0.3; 
xed = 0.6*Ds;
xsd = 0.3*Ds;
Dtime = 0.01;
Fa = 0.15;
Fb = 0.08;
% Foot trajectory
lth_a = LFoot_angle(4*Tc);
rth_a = RFoot_angle(4*Tc);
LZ_a = LFoot_Za(4*Tc);
RZ_a = RFoot_Za(4*Tc);
LX_a = LFoot_Xa(4*Tc);
RX_a = RFoot_Xa(4*Tc);
figure (1)
subplot(3,1,1)
hold on
plot(lth_a(:,1),lth_a(:,2),'-r','LineWidth',2);
plot(lth_a(:,1),rth_a(:,2),'--r','LineWidth',2);
legend('Left Foot','Right Foot','Location','northeast');
title('\theta_{a} Foot');
grid on
subplot(3,1,2)
hold on
plot(RZ_a(:,1),RZ_a(:,2),'-b','LineWidth',2);
plot(RZ_a(:,1),LZ_a(:,2),'--b','LineWidth',2);
legend('Left Foot','Right Foot','Location','northeast');
title('Z_{a} Foot')
grid on

subplot(3,1,3)
hold on
plot(RX_a(:,1),RX_a(:,2),'-g','LineWidth',2);
plot(RX_a(:,1),LX_a(:,2),'--g','LineWidth',2);
legend('Left Foot','Right Foot','Location','northeast');
title('X_{a} Foot')
grid on
Ry = ones(length(RX_a),1);
% Foot= [times LeftX LeftY LeftZ RightX RightY RightZ]
% Foot =[RX_a(:,1),RX_a(:,2),Ry*0.1,RZ_a(:,2),LX_a(:,2),Ry*-0.1,LZ_a(:,2)];
% Foot= [times LeftX RightX LeftZ RightZ LeftTheta RightTheta]
Foot =[RX_a(:,1) RX_a(:,2) LX_a(:,2) RZ_a(:,2) LZ_a(:,2) lth_a(:,2) rth_a(:,2)];
save('.\Matfiles\Foot.mat','Foot');
load('.\Matfiles\ZMP_Ref.mat');
load('.\Matfiles\C_Ref.mat');
% ta_PatternGenneration;
figure(2)
axis equal
view(3)
hold on
grid on
plot3(ZMP_Ref(:,1),ZMP_Ref(:,2),ZMP_Ref(:,3),'r','LineWidth',2);
plot3(C_Ref(:,1),C_Ref(:,2),C_Ref(:,3),'-b','LineWidth',2);
plot3(RX_a(:,2),Ry*0.1,RZ_a(:,2),'-g','LineWidth',2);% Left Foot
plot3(LX_a(:,2),Ry*-0.1,LZ_a(:,2),'-y','LineWidth',2);% Right Foot
tempx = xlim;
tempy = ylim;
tempz = zlim;
axis manual
axis([tempx*1.2 tempy*1.2 tempz*1.4])
daspect([1 1 1])
arrow3([tempx(1) 0 0],[tempx(2) 0 0],'-r1.0',1);
arrow3([0 tempy(1) 0],[0 tempy(2) 0],'-g1.0',1);
arrow3([0 0 tempz(1)],[0 0 tempz(2)*1.3],'-b1.0',1);
text(tempx(2),0,0,'X','FontWeight','bold','color','r');
text(0,tempy(2),0,'Y','FontWeight','bold','color','g');
text(0,0,tempz(2)*1.3,'Z','FontWeight','bold','color','b');

