%%% Descriptions: Tao quy dao
%%% File: UXA_PatternGenerator_demo.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

global Dtime
Dtime = 0.01;


walkstep = 0.05;
HipWidth = 0.114;

% Foot step planning
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

 
Zc = 0.49; % CoM Height
[time,LAnkle,RAnkle,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef,ZMPout] = PatternGenerator(FootStepDesign,Zc);

figure
title('Foot trajectory');
subplot(2,1,1)
hold on
plot(time,RAnkle(:,1),'--k');
plot(time,LAnkle(:,1),'-k');
grid on
legend('Right Leg','Left Leg');
xlabel('time');
ylabel('x(m)')
subplot(2,1,2)
hold on
plot(time,RAnkle(:,3),'--k');
plot(time,LAnkle(:,3),'-k');
grid on
legend('Right Leg','Left Leg');
xlabel('time');
ylabel('z(m)')


figure
subplot(2,1,1)
hold on
plot(time,ZMPRef(:,1),'--r');
plot(time,ZMPout(:,1),'-b');
plot(time,CoM(:,1),'-g');
grid on
legend('Reference','Cart-table', 'CoM');
xlabel('time');
ylabel('x(m)')
subplot(2,1,2)
hold on
plot(time,ZMPRef(:,2),'--r');
plot(time,ZMPout(:,2),'-b');
plot(time,CoM(:,2),'-g');
grid on
legend('Reference','Cart-table', 'CoM');
xlabel('time');
ylabel('y(m)')


figure
title('CoM trajectory');
subplot(2,1,1)
hold on
plot(time,CoM(:,1),'-r');
grid on
xlabel('time');
ylabel('x(m)')
subplot(2,1,2)
hold on
plot(time,CoM(:,2),'-r');
grid on
xlabel('time');
ylabel('y(m)')

