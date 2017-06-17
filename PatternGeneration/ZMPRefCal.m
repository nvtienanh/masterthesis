clear all
close all
clc

% Global variable
% global Tcycle DSrate SSrate SWrate STrate dt
% Tcycle = 1;
% DSrate = 0.2; % Double support
% SSrate = 0.8; % Single support
% SWrate = 0.4; % Swing  phase
% STrate = 0.6; % Stance phase
% dt = 0.01;    % Sampling time of trajectory


walkstep = 0.05;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

ZMPDesign      = [     0     walkstep walkstep walkstep     0;
                  HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];

              
[time,LAnkle,RAnkle,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef,ZMPout]= PatternGenerator(FootStepDesign,0.45);            

figure
hold on
plot(CoM(:,1),CoM(:,2),'-g','Linewidth',1.5);
plot(ZMPRef(:,1),ZMPRef(:,2),'--b','Linewidth',1.5);
xlim([-0.01 0.17]);
grid on
axis equal
xlabel('x(m)');
ylabel('y(m)');
legend('CoM','ZMP_{ref}')

% Plot Ankle Trajectory
figure
subplot(3,1,1)
hold on
plot(time,RAnkle(:,1),'-b')
plot(time,LAnkle(:,1),'-r')
grid on
subplot(3,1,2)
hold on
plot(time,RAnkle(:,2),'-b')
plot(time,LAnkle(:,2),'-r')
grid on
hold off
subplot(3,1,3)
hold on
plot(time,RAnkle(:,3),'-b')
plot(time,LAnkle(:,3),'-r')
grid on
hold off


figure
subplot(2,1,1)
hold on
plot(time,CoM(:,1),'-r');
plot(time,ZMPRef(:,1),'--r');
plot(time,ZMPout(:,1),'-b');
grid on
subplot(2,1,2)
hold on
plot(time,CoM(:,2),'-r');
plot(time,ZMPRef(:,2),'--r');
plot(time,ZMPout(:,2),'-b');
grid on
hold off