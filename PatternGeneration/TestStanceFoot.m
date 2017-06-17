clear all
close all
clc
% UXA_CoMGenerator([0 0 0.5]',1,0.01,1);
%% Walk Phase based on page 34
% J. Rose and J. G. Gamble, Eds., Human walking, 3rd ed. Philadelphia: 
% Lippincott Williams & Wilkins, 2006J. Rose and J. G. Gamble, Eds., Human 
% walking, 3rd ed. Philadelphia: Lippincott Williams & Wilkins, 2006
% Development Phase !! DO NOT MODIFY
walkstep = 0.05;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth HipWidth];                
s_x = FootStepDesign(1,:);
s_y = FootStepDesign(2,:);
ZMPDesign = [     0     walkstep walkstep walkstep walkstep      0;
             HipWidth/2 HipWidth HipWidth HipWidth HipWidth HipWidth/2];
zmp_x = ZMPDesign(1,:);
zmp_y = ZMPDesign(2,:);

[~,NumStep] = size(FootStepDesign);

LeftStanceFirst = 0; % if true: First swing leg is Right Leg

StanceFoot_init = [0 HipWidth/2];
StanceFoot = zeros(NumStep+1,2); % Stance Foot placement Desired
StanceFoot(1,1) = StanceFoot_init(1);
StanceFoot(1,2) = StanceFoot_init(2);

ZMP_init = [0 0];
ZMPRef = zeros(NumStep+1,2); % Stance Foot placement Desired
ZMPRef(1,1) = ZMP_init(1);
ZMPRef(1,2) = ZMP_init(2);

for n = 1:NumStep 
    % Equation 4.50
    StanceFoot(n+1,1) = StanceFoot(n,1) + s_x(n);
    ZMPRef(n+1,1)     = ZMPRef(n,1)     + zmp_x(n);
    if LeftStanceFirst
        StanceFoot(n+1,2) = StanceFoot(n,2) - (-1)^(n)*s_y(n);
        ZMPRef(n+1,2)     = ZMPRef(n,2)     - (-1)^(n)*zmp_y(n);
    else
        StanceFoot(n+1,2) = StanceFoot(n,2) + (-1)^(n)*s_y(n); 
        ZMPRef(n+1,2)     = ZMPRef(n,2)     + (-1)^(n)*zmp_y(n);  
    end    
end

figure
hold on
plot(StanceFoot(:,1),StanceFoot(:,2),'or')
plot(ZMPRef(:,1)    ,ZMPRef(:,2)    ,'xb')
grid on

Tcycle = 1.0;
dt = 0.01; % Sampling time
StartPhase = 'DS';
StopPhase = 'DS';

T_ss = Tcycle/2*0.8; % Single support
T_ds = Tcycle/2*0.2; % Double support
T_sw = Tcycle*0.4;   % Swing  phase
T_st = Tcycle*0.6;   % Stance phase
DSPeriod = 0:dt:T_ds;DSPeriod = DSPeriod'; % Double support
NumPointDS = length(DSPeriod);
SSPeriod = 0:dt:T_ss;SSPeriod = SSPeriod'; % Single support
NumPointSS = length(SSPeriod);
SWPeriod = 0:dt:T_sw;SWPeriod = SWPeriod'; % Swing  phase
NumPointSW = length(SWPeriod);
STPeriod = 0:dt:T_st;STPeriod = STPeriod'; % Stance phase
NumPointST = length(STPeriod);
time = [];

%%% Support state
% SupportState = 1 : Double support phase
% SupportState = 0 : Single support phase
SupportState = 1;
ZMP_x = [];
ZMP_y = [];
current_time = 0;
for idx = 2:NumStep+1
    %%% Double support phase
    zmpx = linspace(ZMPRef(idx-1,1),ZMPRef(idx,1),NumPointDS)';
    zmpy = linspace(ZMPRef(idx-1,2),ZMPRef(idx,2),NumPointDS)';
    
    % Store data
    time =[time;current_time+DSPeriod];
    ZMP_x = [ZMP_x;zmpx];
    ZMP_y = [ZMP_y;zmpy];
    %%% Single support phase
    zmpx = repmat(ZMPRef(idx,1),[NumPointSS,1]);
    zmpy = repmat(ZMPRef(idx,2),[NumPointSS,1]);
    current_time = time(end);
    % Store data
    time =[time;current_time+SSPeriod];
    ZMP_x = [ZMP_x;zmpx];
    ZMP_y = [ZMP_y;zmpy];
    current_time = time(end);
end

WalkkingTime = time(end); 

%% Remove duplicate value in time series
NumData = length(time);
DuplicateIdx = [];
for idx = 1:NumData-1
    if time(idx) == time(idx+1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end
time(DuplicateIdx)=[];
ZMP_x(DuplicateIdx) = [];
ZMP_y(DuplicateIdx) = [];
%%% ----------------------------------------

%% Foot trajectory
% LFeetStance = zeros(3,3);
% RFeetStance = zeros(3,3);
% idxL = 0;
% idxR = 0;
% for idx = 1:6
%     if StanceFoot(idx,2) > 0
%         idxL = idxL+1;
%         LFeetStance(idxL,1) = StanceFoot(idx,1);
%         LFeetStance(idxL,2) = StanceFoot(idx,2);
%     else
%         idxR = idxR+1;
%         RFeetStance(idxR,1) = StanceFoot(idx,1);
%         RFeetStance(idxR,2) = StanceFoot(idx,2);
%     end
% end
LFeetStance = [];
RFeetStance = [];

if LeftStanceFirst
    LeftFlag = 0;
else
    LeftFlag = 1;
end

for idx = 1:NumStep+1
    if LeftFlag
        LFeetStance = [LFeetStance;[StanceFoot(idx,:) 0]];
    else
        RFeetStance = [RFeetStance;[StanceFoot(idx,:) 0]];
    end
    LeftFlag = bitxor(LeftFlag,1);
end

%% Right Leg stance first
timeRFeet = 0;
RFeet = RFeetStance(1,:);
[row,~] = size(RFeetStance);
for idx = 2:row
    % Stance phase
    timeRFeet = [timeRFeet;timeRFeet(end) + T_st];    
    RFeet = [RFeet;RFeetStance(idx-1,:)];
    % Swing phase
    timeRFeet = [timeRFeet;timeRFeet(end)+ T_sw];
    RFeet = [RFeet;RFeetStance(idx,:)];
end
%%%------------------

%% Left Leg swing first

timeLFeet = 0;
LFeet = [];
[row,~] = size(LFeetStance);
for idx = 2:row    
    % Swing phase
    timeLFeet = [timeLFeet;timeLFeet(end) + T_sw];    
    LFeet = [LFeet;LFeetStance(idx-1,:)];    
    % Stance phase
    timeLFeet = [timeLFeet;timeLFeet(end) + T_st];   
    LFeet = [LFeet;LFeetStance(idx,:)];    
end
LFeet = [LFeet;LFeetStance(row,:)];
%%%------------------

%% Create Ankle data for Right leg
timeR = [];
AnkleR = [];
current_time = 0;
for idx = 1:length(RFeet)-1
    if (RFeet(idx,1) == RFeet(idx+1,1))
        % Right Leg is in Stance phase
        timeR = [timeR;current_time+STPeriod];
        AnkleR = [AnkleR;repmat(RFeet(idx,:),size(STPeriod))];
        current_time = timeR(end);
    else
        % Right Leg is in Swing phase
        timeR = [timeR;current_time+SWPeriod];        
        [~,pos_X] = UXA_Ankle_X(RFeet(idx,1),RFeet(idx+1,1),timeRFeet(idx),timeRFeet(idx+1),0.01);        
        [~,pos_Z] = UXA_Ankle_Z(0.1,timeRFeet(idx),timeRFeet(idx+1),0.01);
        pos_Y = repmat(RFeet(idx,2),size(SWPeriod));
        AnkleR = [AnkleR;[pos_X pos_Y pos_Z]];
        current_time = timeR(end);
    end
end
% Adding final state of Right
% when robot stop: both feet in double support phase
tmpTime = (timeR(end):dt:WalkkingTime)';
timeR = [timeR;tmpTime];
AnkleR = [AnkleR;repmat(AnkleR(end,:),size(tmpTime))];

%%% It OK!
%%%----------------------------------

%% Create Ankle data for Left leg
timeL = [];
AnkleL = [];
current_time = 0;
for idx = 1:length(LFeet)-1
    if (LFeet(idx,1) == LFeet(idx+1,1))
        % Left Leg is in Stance phase
        timeL = [timeL;current_time+STPeriod];
        AnkleL = [AnkleL;repmat(LFeet(idx,:),size(STPeriod))];
        current_time = timeL(end);
    else
        % Left Leg is in Swing phase
        timeL = [timeL;current_time+SWPeriod];        
        [~,pos_X] = UXA_Ankle_X(LFeet(idx,1),LFeet(idx+1,1),timeLFeet(idx),timeLFeet(idx+1),0.01);        
        [~,pos_Z] = UXA_Ankle_Z(0.1,timeLFeet(idx),timeLFeet(idx+1),0.01);
        pos_Y = repmat(LFeet(idx,2),size(SWPeriod));
        AnkleL = [AnkleL;[pos_X pos_Y pos_Z]];
        current_time = timeL(end);
    end
end
% Vi tri ban dau hai chan o vi tri double support
% Vi then trong khoang thoi gian 0 ->T_ds, chan trai dung yen
% Bo sung them khoang thoi gian do vao chan trai
% Xem them hinh anh human walking giat (search google)
LeftFeet_Init = repmat(LFeetStance(1,:),size(DSPeriod));
LeftTime_Init = DSPeriod;
AnkleL = [LeftFeet_Init;AnkleL];
timeL = [LeftTime_Init;LeftTime_Init(end)+timeL];

%%% Thoi gian tao quy dao ZMP, LeftFoot, RightFoot la khac nhau  vi vay can
%%% phai dong bo hoa thoi gia cua 3 quy dao duoc tao ra
time_end = [time(end) timeL(end) timeR(end)];
[max_time,~] = max(time_end);

tmpTime = (timeR(end):dt:max_time)';
timeR = [timeR;tmpTime];
AnkleR = [AnkleR;repmat(AnkleR(end,:),size(tmpTime))];
% Adding final state of Left Leg
% when robot stop: both feet in double support phase
tmpTime = (timeL(end):dt:max_time)';
timeL = [timeL;tmpTime];
AnkleL = [AnkleL;repmat(AnkleL(end,:),size(tmpTime))];

tmpTime = (time(end):dt:max_time)';
time = [time;tmpTime];
ZMP_x = [ZMP_x;repmat(ZMP_x(end,:),size(tmpTime))];
ZMP_y = [ZMP_y;repmat(ZMP_y(end,:),size(tmpTime))];
%%% It OK!
%%%----------------------------------

%% Remove duplicate value in time series of data
NumData = length(timeL);
DuplicateIdx = [];
for idx = 1:NumData-1
    if timeL(idx) == timeL(idx+1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end
timeL(DuplicateIdx)=[];
AnkleL(DuplicateIdx,:) = [];
%---
NumData = length(timeR);
DuplicateIdx = [];
for idx = 1:NumData-1
    if timeR(idx) == timeR(idx+1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end
timeR(DuplicateIdx)=[];
AnkleR(DuplicateIdx,:) = [];

%%% ----------------------------------------
%%% Plot ankle trajectory
figure
title('Ankle Trajacetory')
subplot(3,1,1)
hold on
plot(timeR,AnkleR(:,1),'-r');
plot(timeL,AnkleL(:,1),'-b')
legend('Right','Left');
grid on
title('X direction');
subplot(3,1,2);
hold on
plot(timeR,AnkleR(:,2),'-r');
plot(timeL,AnkleL(:,2),'-b');
legend('Right','Left');
title('Y direction');
grid on
subplot(3,1,3);
hold on
plot(timeR,AnkleR(:,3),'-r');
plot(timeL,AnkleL(:,3),'-b');
legend('Right','Left');
title('Z direction');
grid on

% X = UXA_Ankle(xstart,xend,tstart,tend)
% figure
% subplot(3,1,1)
% plot(time,ZMP_x,'-r');
% grid on
% subplot(3,1,2)
% plot(time,ZMP_y,'-r');
% grid on
% subplot(3,1,3)
% plot(ZMP_x,ZMP_y,'-r');
% grid on
% 
ZMPDeseried = [ZMP_x ZMP_y];
[CoM,dCoM]= CalCoMbasedCartTable( ZMPDeseried,dt,0.52);

figure
title('ZMP and CoM Trajacetory')
subplot(2,1,1)
hold on
plot(time,ZMPDeseried(:,1),'-r')
plot(time,CoM(:,1),'-b')
grid on
title('X direction')
legend('ZMP','CoM');
subplot(2,1,2)
hold on
plot(time,ZMPDeseried(:,2),'-r')
plot(time,CoM(:,2),'-b');
legend('ZMP','CoM');
title('Y direction')

save Time.mat time
save CoMSim.mat CoM
save AnkleL.mat AnkleL
save AnkleR.mat AnkleR
save dCoMSim.mat dCoM
% 
% figure
% subplot(2,1,1)
% hold on
% plot(time,dCoM(:,1),'-b')
% subplot(2,1,2)
% hold on
% plot(time,dCoM(:,2),'-b')
