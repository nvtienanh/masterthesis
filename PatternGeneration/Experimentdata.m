clear all
close all
clc
addpath('First_4Step')
addpath('Matfile');
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

              
[time,LAnkle_bk,RAnkle_bk,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef_bk,ZMPout_bk] = PatternGenerator(FootStepDesign,0.45);            


figure
subplot(2,1,1)
hold on
plot(time,CoM(:,1),'-r');
plot(time,ZMPRef_bk(:,1),'--r');
plot(time,ZMPout_bk(:,1),'-b');
grid on
subplot(2,1,2)
hold on
plot(time,CoM(:,2),'-r');
plot(time,ZMPRef_bk(:,2),'--r');
plot(time,ZMPout_bk(:,2),'-b');
grid on
hold off

%% CALCULATE ZMP STABLE MARGIN
zmp_bound_x = zeros(length(LAnkle_bk),2);
zmp_bound_y = zeros(length(LAnkle_bk),2);
for idx=1:length(LAnkle_bk)
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
%%%%----------------------------------------------------------------------

%%% Experiment data
load('LeftData.mat');
load('RightData.mat');
load ('ZMP_m.mat');
ZMP_m_bk = ZMP_m;
% Event Timeline: Period when a 4STEP motion happen
% Time =
% 240-250s
% 270-280s
% 290-300s
% 310-315s
% 328-332s
% ~346s

t_begin_l=24000;
t_end_l=25000;

t_begin_r=23000;
t_end_r=24500;

 
timeL = LeftData{7}(:,1);
timeR = RightData{7}(:,1);
ZMPL = LeftData{7}(:,2);
ZMPR = RightData{7}(:,2);

t_L = [20490 20975;
       24190 24690;
       26787 27285;
       29387 29887;
       30685 31189;
       32389 32988;
       33780 34285];
   
t_R = [20490 20975;
       24190 24690;
       26787 27285;
       29387 29887;
       30685 31189;
       32389 32987;
       33780 34285];
offset = [-676 -718;% +325 442
       -719 -775;
       -775 -776;
       -789 -823;
       -824 -850;
       -851 -862;
       -862 -865];
t_R = t_R + offset;   

%%% Plot all sample data
[row,~]= size(t_L);
dt = 0.01;
for tt = 1:row
    LAnkle = LAnkle_bk;
    RAnkle = RAnkle_bk;
    ZMPRef = ZMPRef_bk;
    ZMP_m = ZMP_m_bk;
    ZMPout = ZMPout_bk;
    % ZMP stable region
    zmp_x = zmp_bound_x;
    zmp_y = zmp_bound_y;
    %-----
    timeL = LeftData{7}(t_L(tt,1):t_L(tt,2),1);
    timeL = timeL - timeL(1);
    timeR = RightData{7}(t_R(tt,1):t_R(tt,2),1);
    timeR = timeR - timeR(1);

    ZMPL = LeftData{7}(t_L(tt,1):t_L(tt,2),2);
    ZMPR = RightData{7}(t_R(tt,1):t_R(tt,2),2);
    ZMPL_x = LeftData{5}(t_L(tt,1):t_L(tt,2),2);
    ZMPR_x = RightData{5}(t_R(tt,1):t_R(tt,2),2);
    ZMPL_y = LeftData{6}(t_L(tt,1):t_L(tt,2),2);
    ZMPR_y = RightData{6}(t_R(tt,1):t_R(tt,2),2);
    
    %%% dong bo thoi gian
    [~, ZMPL] = even_sample(timeL, ZMPL, 1/dt);
    [~, ZMPL_x] = even_sample(timeL, ZMPL_x, 1/dt);
    [~, ZMPL_y] = even_sample(timeL, ZMPL_y, 1/dt);
% 
    [~, ZMPR] = even_sample(timeR, ZMPR, 1/dt);
    [~, ZMPR_x] = even_sample(timeR, ZMPR_x, 1/dt);
    [~, ZMPR_y] = even_sample(timeR, ZMPR_y, 1/dt);
% 
    time = zeros(length(ZMPL),1);

    for n = 2:length(ZMPL)
        time(n) = time(n-1)+ dt;
    end
%     ZMPL = dtrend(ZMPL,1);
%     ZMPR = dtrend(ZMPR,1);
%     figure
%     hold on
%     plot(time,ZMPL,'-r');
%     plot(time,ZMPR,'-b');
%     grid on
%     hold off
%     
    
    % position in world coordinate
%     ZMPL_x = -(HipWidth/2*1000+FootWidth/2-ZMPL_x);
%     ZMPR_x = (HipWidth/2*1000-FootWidth/2+ZMPR_x);
    ZMPL_x = -(HipWidth/2*1000+(FootWidth+0)/2-ZMPL_x);
    ZMPR_x = (HipWidth/2*1000-(FootWidth-50)/2+ZMPR_x);


    % offset value to match value between simulation and experiment
    offset = 80;
    idxdata = offset + [13 73 85 145 157 217 229 289 301]';

    ZMPX = zeros(size(time));
    ZMPY = zeros(size(time));


    %%% Match experiment and simulation time
    oslen = 10;
    tmp = repmat(RAnkle(1,:),[oslen,1]);
    RAnkle = [tmp;RAnkle];
    tmp = repmat(LAnkle(1,:),[oslen,1]);
    LAnkle = [tmp;LAnkle];
    tmp = repmat(ZMPRef(1,:),[oslen,1]);
    ZMPRef = [tmp;ZMPRef];
    tmp = repmat(ZMP_m(1,:),[oslen,1]);
    ZMP_m = [tmp;ZMP_m];
    tmp = repmat(zmp_x(1,:),[oslen,1]);
    zmp_x = [tmp;zmp_x];
    tmp = repmat(zmp_y(1,:),[oslen,1]);
    zmp_y = [tmp;zmp_y];
    tmp = repmat(ZMPout(1,:),[oslen,1]);
    ZMPout = [tmp;ZMPout];

    %%% Match experiment and simulation time
    tmp = repmat(RAnkle(end,:),[length(ZMPX)-length(RAnkle),1]);
    RAnkle = [RAnkle;tmp];
    tmp = repmat(LAnkle(end,:),[length(ZMPX)-length(LAnkle),1]);
    LAnkle = [LAnkle;tmp];
    tmp = repmat(ZMPRef(end,:),[length(ZMPX)-length(ZMPRef),1]);
    ZMPRef = [ZMPRef;tmp];

    tmp = repmat(ZMP_m(end,:),[length(ZMPX)-length(ZMP_m),1]);
    ZMP_m = [ZMP_m;tmp];
    tmp = repmat(zmp_x(end,:),[length(ZMPX)-length(zmp_x),1]);
    zmp_x = [zmp_x;tmp];
    tmp = repmat(zmp_y(end,:),[length(ZMPX)-length(zmp_y),1]);
    zmp_y = [zmp_y;tmp];
    tmp = repmat(ZMPout(end,:),[length(ZMPX)-length(ZMPout),1]);
    ZMPout = [ZMPout;tmp];

    cnt = 1;
    % Double support
    for idx = 1:idxdata(cnt)-1
        ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
        ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
    end
    % Single support right leg
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
    %     ZMPX(idx) = ZMPR_x(idx);
        ZMPX(idx) = ZMPL_x(idx);
        ZMPY(idx) = ZMPR_y(idx) + RAnkle(idx,1)*1000;
    end
    % Double support,
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
        ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
    end
    % Single support Left leg
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx) = ZMPR_x(idx);
    %     ZMPX(idx) = ZMPL_x(idx);
        ZMPY(idx) = ZMPL_y(idx) + LAnkle(idx,1)*1000;
    end
    % Double support
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
        ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
    end
    % Single support Right leg 
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx)= ZMPL_x(idx);
    %     ZMPX(idx)= ZMPR_x(idx);
        ZMPY(idx) = ZMPR_y(idx) + RAnkle(idx,1)*1000;
    end
    % Double support
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
        ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
    end
    % Single support Left leg
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):idxdata(cnt)-1
        ZMPX(idx) = ZMPR_x(idx);
    %     ZMPX(idx) = ZMPL_x(idx);
        ZMPY(idx) = ZMPL_y(idx) + LAnkle(idx,1)*1000;
    end
    % Double support
    cnt = cnt + 1;
    for idx = idxdata(cnt-1):length(time)
        ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
        ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
    end

    % ZMP in word coordinate: y direction
%     ZMPX = dtrend(ZMPX);
    figure
    subplot(2,1,1)
    hold on
    plot(time, ZMPX,'-g','LineWidth',1);
    plot(time,ZMPout(:,2)*1000,'-r','LineWidth',1);
    plot(time,ZMP_m(:,2)*1000,'-b','LineWidth',1);
    plot(time,zmp_y(:,1)*1000,'--k');
    plot(time,zmp_y(:,2)*1000,'--k');
    ylim([-120 120])
    xlim([0 5])
    xlabel('time');
    ylabel('y(mm)');
    legend('ZMP sensor','ZMP Cart-table','ZMP Multiboldy');
    grid on
    subplot(2,1,2)
    hold on
    plot(time, ZMPY - ZMPY(1),'-g','LineWidth',1);
    plot(time,ZMPout(:,1)*1000,'-r','LineWidth',1);
    plot(time,ZMP_m(:,1)*1000,'-b','LineWidth',1);
    plot(time,zmp_x(:,1)*1000,'--k');
    plot(time,zmp_x(:,2)*1000,'--k');
    xlabel('time');
    ylabel('x(mm)');
    ylim([-110 260])
    xlim([0 5])
    legend('ZMP sensor','ZMP Cart-table','ZMP Multiboldy');
    grid on
    
end