clear all
close all
clc
addpath('First_4Step')

% Global variable
% global Tcycle DSrate SSrate SWrate STrate dt
% Tcycle = 1;
% DSrate = 0.2; % Double support
% SSrate = 0.8; % Single support
% SWrate = 0.4; % Swing  phase
% STrate = 0.6; % Stance phase
% dt = 0.01;    % Sampling time of trajectory

FootWidth = 71;
FootHeight = 200;

walkstep = 0.05;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

ZMPDesign      = [     0     walkstep walkstep walkstep     0;
                  HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];

              
[time,LAnkle_bk,RAnkle_bk,CoM,dCoM,ZMPRef_bk,ZMPout] = PatternGenerator(FootStepDesign,0.45);  

% SupportPol = zeros(length(RAnkle_bk,2));

%% CALCULATE ZMP STABLE MARGIN
zmp_bound_x = zeros(length(time),2);
zmp_bound_y = zeros(length(time),2);
for idx=1:length(time)
    if RAnkle_bk(idx,3)<=0 && LAnkle_bk(idx,3)<=0 % Double support phase
        zmp_bound_x(idx,1) = min([LAnkle_bk(idx,1)-FootHeight/2/1000;LAnkle_bk(idx,1)+FootHeight/2/1000;...
            RAnkle_bk(idx,1)-FootHeight/2/1000;RAnkle_bk(idx,1)+FootHeight/2/1000]);
        zmp_bound_x(idx,2) = max([LAnkle_bk(idx,1)-FootHeight/2/1000;LAnkle_bk(idx,1)+FootHeight/2/1000;...
            RAnkle_bk(idx,1)-FootHeight/2/1000;RAnkle_bk(idx,1)+FootHeight/2/1000]);
        zmp_bound_y(idx,1) = min([LAnkle_bk(idx,2)-FootWidth/2/1000;LAnkle_bk(idx,2)+FootWidth/2/1000;...
            RAnkle_bk(idx,2)-FootWidth/2/1000;RAnkle_bk(idx,2)+FootWidth/2/1000]);
        zmp_bound_y(idx,2) = max([LAnkle_bk(idx,2)-FootWidth/2/1000;LAnkle_bk(idx,2)+FootWidth/2/1000;...
            RAnkle_bk(idx,2)-FootWidth/2/1000;RAnkle_bk(idx,2)+FootWidth/2/1000]);
    elseif RAnkle_bk(idx,3)>0 % Left Leg is stance
        zmp_bound_x(idx,1) = LAnkle_bk(idx,1)-FootHeight/2/1000;                                  
        zmp_bound_x(idx,2) = LAnkle_bk(idx,1)+FootHeight/2/1000;
        zmp_bound_y(idx,1) = LAnkle_bk(idx,2)-FootWidth/2/1000;                                  
        zmp_bound_y(idx,2) = LAnkle_bk(idx,2)+FootWidth/2/1000;
    elseif LAnkle_bk(idx,3)>0 % Right Leg stance
        zmp_bound_x(idx,1) = RAnkle_bk(idx,1)-FootHeight/2/1000;                                  
        zmp_bound_x(idx,2) = RAnkle_bk(idx,1)+FootHeight/2/1000;
        zmp_bound_y(idx,1) = RAnkle_bk(idx,2)-FootWidth/2/1000;                                  
        zmp_bound_y(idx,2) = RAnkle_bk(idx,2)+FootWidth/2/1000;
    else
        display('Error---------')
    end 
end

figure
subplot(2,1,1)
hold on
plot(time,CoM(:,1),'-r');
plot(time,ZMPRef_bk(:,1),'--r');
plot(time,zmp_bound_x(:,1),'--k');
plot(time,zmp_bound_x(:,2),'--k');

grid on
subplot(2,1,2)
hold on
plot(time,CoM(:,2),'-r');
plot(time,ZMPRef_bk(:,2),'--r');
plot(time,zmp_bound_y(:,1),'--k');
plot(time,zmp_bound_y(:,2),'--k');
grid on
hold off