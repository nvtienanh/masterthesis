% FootTrajectory.m
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
close all
clear
clc
addpath ../
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
t = 4*Tc; % Time simulation
len = length(0:Dtime:t);
%% Foot trajectory
% Ankle Angle of Left Foot
    k = 0;
    AnkleAngle_time=[k*Tc;k*Tc+Td;(k+1)*Tc;(k+1)*Tc+Td];
    AnkleAngle_value=[qs;qb;qf;qe];
    t1 = AnkleAngle_time(1);
    t2 = AnkleAngle_time(2);
    t3 = AnkleAngle_time(3);
    t4 = AnkleAngle_time(4);
    f1 = AnkleAngle_value(1);
    f2 = AnkleAngle_value(2);
    f3 = AnkleAngle_value(3);
    f4 = AnkleAngle_value(4);
    A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0;...
       t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0;...
       0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1;...
       0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
       3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0;...
       3*t2^2,2*t2,1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,3*t2^2,2*t2,1,0,0,0,0,0;...
       0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t3^2,2*t3,1,0;...
       0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0];
    Y = [f1;f2;f2;f3;f3;f4;0;0;0;0;0;0];
    X = A\Y;
    AnkleAngleFactor = zeros(3,4);
    EXP_TH=0;
    for i=1:3
        AnkleAngleFactor(i,:)=[X(EXP_TH+1),X(EXP_TH+2),X(EXP_TH+3),X(EXP_TH+4)];
        EXP_TH = EXP_TH+4;    
    end
    WalkingTimes=[];
    AnkleAngle_L=[];
    for i=1:length(AnkleAngle_time)-1
        y_1 = @(x) AnkleAngleFactor(i,1)*x.^3+...
            AnkleAngleFactor(i,2)*x.^2+AnkleAngleFactor(i,3)*x+...
            AnkleAngleFactor(i,4);
        x_1 = AnkleAngle_time(i):Dtime:AnkleAngle_time(i+1);
         % Remove repeat value
        if i<length(AnkleAngle_time)-1
            x_1(end)=[];
        end
        WalkingTimes=[WalkingTimes;x_1'];
        temp = y_1(x_1);
        AnkleAngle_L=[AnkleAngle_L;temp'];        
    end
    temp = WalkingTimes(end):Dtime:2*Tc;
    temp(1)=[];
    temp(end)=[];
    WalkingTimes=[WalkingTimes;temp'];
    AnkleAngle_L=[AnkleAngle_L;zeros(length(temp),1)];
    % For right Foot
    AnkleAngle_R = AnkleAngle_L;
    idx = find(WalkingTimes==Tc);    
    tmp = [AnkleAngle_R(idx:end);AnkleAngle_R(1:idx-1)];
    AnkleAngle_R = tmp;
    clear tmp
    
    numCycle = round(t/2/Tc+0.5);
    oneCycle = WalkingTimes;
    oneCycle_R = oneCycle;
    oneAngle_L = AnkleAngle_L;
    oneAngle_R = AnkleAngle_R;
    % Repeat for other Cycles Left Foot    
    for i=2:numCycle
        oneCycle = 2*Tc+oneCycle;
        temp_time = oneCycle;
        temp_time(1)=[];
        WalkingTimes=[WalkingTimes;temp_time];
        temp_L = oneAngle_L;
        temp_L(1)=[];
        AnkleAngle_L = [AnkleAngle_L;temp_L];
        temp_R = oneAngle_R;
        temp_R(1)=[];
        AnkleAngle_R = [AnkleAngle_R;temp_R];
    end

    WalkingTimes = WalkingTimes(1:len+1);
    AnkleAngle_L = AnkleAngle_L(1:len+1);
    AnkleAngle_R = AnkleAngle_R(1:len+1);
    clear tmp_time tmp_L tmp_R

% Ankle X

AnkleX_value=[k*Ds;...
            k*Ds+laf*(1-cos(qb))+lan*sin(qb);...
            k*Ds+Lao;...
            (k+1)*Ds-lab*(1-cos(qf))-lan*sin(qf);...
            (k+1)*Ds];
AnkleX_time=[k*Tc;...
            k*Tc+Td;...
            k*Tc+To;...
            (k+1)*Tc;...
            (k+1)*Tc+Td];
    % X_a foot;
    % AX=y
    t1 = AnkleX_time(1);
    t2 = AnkleX_time(2);
    t3 = AnkleX_time(3);
    t4 = AnkleX_time(4);
    t5 = AnkleX_time(5);
    f1 = AnkleX_value(1);
    f2 = AnkleX_value(2);
    f3 = AnkleX_value(3);
    f4 = AnkleX_value(4);
    f5 = AnkleX_value(5);
    A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
       0,0,0,0,0,0,0,0,0,0,0,0,t5^3,t5^2,t5,1;...
       3*t2^2,2*t2,1,0,-3*t2^2,-2*t2,-1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,3*t3^2,2*t3,1,0,-3*t3^2,-2*t3,-1,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0,-3*t4^2,-2*t4,-1,0;...
       3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0,0,0,0,0;...
       6*t2,2,0,0,-6*t2,-2,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,6*t3,2,0,0,-6*t3,-2,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,6*t4,2,0,0,-6*t4,-2,0,0;...      
       0,0,0,0,0,0,0,0,0,0,0,0,3*t5^2,2*t5,1,0];
    Y = [f1;f2;f2;f3;f3;f4;f4;f5;0;0;0;0;0;0;0;0];
    X = A\Y;
    AnkleXFactor = zeros(length(AnkleX_time)-1,4);
    EXP_X=0;
    for i=1:length(AnkleX_time)-1;
        AnkleXFactor(i,:)=[X(EXP_X+1),X(EXP_X+2),X(EXP_X+3),X(EXP_X+4)];
        EXP_X = EXP_X+4;    
    end
    
    AnkleX_L=[];
    AnkleX_times=[];
    for i=1:length(AnkleX_time)-1
        y_1 = @(x) AnkleXFactor(i,1)*x.^3+AnkleXFactor(i,2)*x.^2+...
            AnkleXFactor(i,3)*x+AnkleXFactor(i,4);
%       x_1 = AnkleX_time(i):Dtime:AnkleX_time(i+1);
        x_1 = WalkingTimes(WalkingTimes<AnkleX_time(i+1)...
            & WalkingTimes>=AnkleX_time(i));
        % Remove repeat value
        if i<length(AnkleX_time)-1
            x_1(end)=[];
        end
        AnkleX_times=[AnkleX_times;x_1];
%         temp = y_1(x_1);
        AnkleX_L=[AnkleX_L;y_1(x_1)];
    end
    temp = WalkingTimes(WalkingTimes<2*Tc...
            & WalkingTimes>AnkleX_times(end));
    AnkleX_times=[AnkleX_times;temp];
    AnkleX_L=[AnkleX_L;ones(length(temp),1)*Ds];
    % For right Foot
    AnkleX_R = AnkleX_L;
    Xa=[Xa;ones(length(temp),1)*Ds];
    oneCycle_X = AnkleX_times;
    idx = find(AnkleX_times==Tc);    
    tmp = [AnkleX_L(idx:end)-0.5*Ds;AnkleX_L(1:idx-1)+0.5*Ds];
    AnkleX_L = tmp;
    oneX = AnkleX_L;
    clear tmp
    for i=2:numCycle
        oneCycle_X = 2*Tc+oneCycle_X;
        oneX = oneX+Ds;
        temp = oneX;
        temp(1)=[];
        AnkleX_L = [AnkleX_L;temp];
    end
    AnkleX_L = AnkleX_L(1:len+1);

% Ankle Z
%% End Foot trajectory
% LZ_a = LFoot_Za(4*Tc);
% RZ_a = RFoot_Za(4*Tc);
% LX_a = LFoot_Xa(4*Tc);
% RX_a = RFoot_Xa(4*Tc);
figure (1)
subplot(3,1,1)
hold on
plot(WalkingTimes,AnkleAngle_L,'-r','LineWidth',2);
plot(WalkingTimes,AnkleAngle_R,'--r','LineWidth',2);
legend('Left Foot','Right Foot','Location','northeast');
title('\theta_{a} Foot');
grid on
subplot(3,1,2)
hold on
plot(WalkingTimes,AnkleX_L,'-b','LineWidth',2);
% plot(RZ_a(:,1),LZ_a(:,2),'--b','LineWidth',2);
% legend('Left Foot','Right Foot','Location','northeast');
title('Z_{a} Foot')
grid on

% subplot(3,1,3)
% hold on
% plot(RX_a(:,1),RX_a(:,2),'-g','LineWidth',2);
% plot(RX_a(:,1),LX_a(:,2),'--g','LineWidth',2);
% legend('Left Foot','Right Foot','Location','northeast');
% title('X_{a} Foot')
% grid on
% Foot =[RX_a(:,1) RX_a(:,2) LX_a(:,2) RZ_a(:,2) LZ_a(:,2) rth_a(:,2) lth_a(:,2)];
% save('Foot.mat','Foot');
% load ZMP_Ref.mat
% load C_Ref.mat
% % ta_PatternGenneration;
% Ry = ones(length(RX_a),1);
% figure(2)
% axis equal
% view(3)
% hold on
% grid on
% plot3(ZMP_Ref(:,1),ZMP_Ref(:,2),ZMP_Ref(:,3),'r');
% plot3(C_Ref(:,1),C_Ref(:,2),C_Ref(:,3),'-b');
% plot3(RX_a(:,2),Ry*0.1,RZ_a(:,2),'-g');
% plot3(LX_a(:,2),Ry*-0.1,LZ_a(:,2),'-y');
% tempx = xlim;
% tempy = ylim;
% tempz = zlim;
% axis manual
% axis([tempx*1.2 tempy*1.2 tempz*1.4])
% daspect([1 1 1])
% arrow3([tempx(1) 0 0],[tempx(2) 0 0],'-r1.0',1);
% arrow3([0 tempy(1) 0],[0 tempy(2) 0],'-g1.0',1);
% arrow3([0 0 tempz(1)],[0 0 tempz(2)*1.3],'-b1.0',1);
% text(tempx(2),0,0,'X','FontWeight','bold','color','r');
% text(0,tempy(2),0,'Y','FontWeight','bold','color','g');
% text(0,0,tempz(2)*1.3,'Z','FontWeight','bold','color','b');

