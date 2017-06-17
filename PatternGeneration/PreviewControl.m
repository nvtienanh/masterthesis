clear all
close all
clc


walkstep = 0.08;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

ZMPDesign      = [     0     walkstep walkstep walkstep     0;
                  HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];

ZMPDesign = FootStepDesign;
ZMPDesign(2,1) = ZMPDesign(2,1)/2;
ZMPDesign(2,end) = ZMPDesign(2,end)/2;

global Tcycle DSrate SSrate SWrate STrate dt
Tcycle = 1;
DSrate = 0.2; % Double support
SSrate = 0.8; % Single support
SWrate = 0.4; % Swing  phase
STrate = 0.6; % Stance phase
dt = 0.01;    % Sampling time of trajectory


ZMPRef = CalZMPRef(ZMPDesign,[0 0],'Left');
DuplicateIdx = [];
for idx = 1: length(ZMPRef)-1
    if (ZMPRef(idx,1) == ZMPRef(idx+1,1))
        DuplicateIdx = [DuplicateIdx;idx];
    end
end
ZMPRef(DuplicateIdx,:)=[];

% [cog_x,cog_y,output_zmp_x,output_zmp_y] = calc_preview_control(ZMPRef(:,2),ZMPRef(:,3),ZMPRef(end,1),1);
[CoM,dCoM,ZMPout] = calc_preview_control(ZMPRef(:,2:3),0.45,ZMPRef(end,1),10,dt);
% [CoM,dCoM,ZMPout]= calc_preview_control(ZMP_xy,Zcom,calc_time,preview_time,dt)
ZMPRef(end,1)

figure
subplot(2,1,1)
hold on
plot(ZMPRef(:,1),ZMPRef(:,2),'--r')
plot(ZMPRef(:,1),ZMPout(:,1),'-b')
plot(ZMPRef(:,1),CoM(:,1),'-r')
grid on
subplot(2,1,2)
hold on
plot(ZMPRef(:,1),ZMPRef(:,3),'--r')
plot(ZMPRef(:,1),ZMPout(:,2),'-b')
plot(ZMPRef(:,1),CoM(:,2),'-r')
grid on


figure
subplot(2,1,1)
plot(ZMPRef(:,1),dCoM(:,1),'-r');
grid on
subplot(2,1,2)
plot(ZMPRef(:,1),dCoM(:,2),'-r');
grid on
