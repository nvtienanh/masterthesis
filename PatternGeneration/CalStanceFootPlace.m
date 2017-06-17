function [LAnkle,RAnkle,dAnkleL,dAnkleR] = CalStanceFootPlace(WalkingParameter,Init_pos,SwingFirst)
%CalStanceFootPlace Fucntion calculate StanceFoot Place ment
%   Detailed explanation goes here
global Tcycle DSrate SWrate STrate dt stepHeight               
s_x = WalkingParameter(1,:);
s_y = WalkingParameter(2,:);

[~,NumStep] = size(WalkingParameter);

if strcmp(SwingFirst,'Left')
    LeftStanceFirst = 0;
else
    LeftStanceFirst = 1;
end

StanceFoot = zeros(NumStep+1,2); % Stance Foot placement Desired
StanceFoot(1,1) = Init_pos(1);
StanceFoot(1,2) = Init_pos(2);

for n = 1:NumStep 
    % Equation 4.50
    StanceFoot(n+1,1) = StanceFoot(n,1) + s_x(n);    
    if LeftStanceFirst
        StanceFoot(n+1,2) = StanceFoot(n,2) - (-1)^(n)*s_y(n);        
    else
        StanceFoot(n+1,2) = StanceFoot(n,2) + (-1)^(n)*s_y(n);          
    end    
end

%%% ----------------------------------------
T_ds = Tcycle/2*DSrate; % Double support
T_sw = Tcycle*SWrate;   % Swing  phase
T_st = Tcycle*STrate;   % Stance phase
DSPeriod = 0:dt:T_ds;DSPeriod = DSPeriod'; % Double support
SWPeriod = 0:dt:T_sw;SWPeriod = SWPeriod'; % Swing  phase
STPeriod = 0:dt:T_st;STPeriod = STPeriod'; % Stance phase

%% Calculate number of stance phase both feet
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

% figure
% hold on
% plot(RFeetStance(:,1),RFeetStance(:,2),'or');
% plot(LFeetStance(:,1),LFeetStance(:,2),'ob');
% grid on
% axis equal
% hold off

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
%%%------------------
% display(LFeet);
%% Create Ankle data for Right leg
timeR = [];
AnkleR = [];
dAnkleR = [];
current_time = 0;
for idx = 1:length(RFeet)-1
    if (RFeet(idx,1) == RFeet(idx+1,1))
        % Right Leg is in Stance phase
        timeR = [timeR;current_time+STPeriod];
        AnkleR = [AnkleR;repmat(RFeet(idx,:),size(STPeriod))];
        dAnkleR = [dAnkleR;repmat([0 0 0],size(STPeriod))];
        current_time = timeR(end);
    else
        % Right Leg is in Swing phase
        timeR = [timeR;current_time+SWPeriod];        
        [~,pos_X,vec_X] = UXA_Ankle_X(RFeet(idx,1),RFeet(idx+1,1),timeRFeet(idx),timeRFeet(idx+1),0.01);        
        [~,pos_Z,vec_Z] = UXA_Ankle_Z(stepHeight,timeRFeet(idx),timeRFeet(idx+1),0.01);
        pos_Y = repmat(RFeet(idx,2),size(SWPeriod));
        vec_Y = zeros(size(pos_Y));
%         length(pos_Y)
%         length(pos_X)
%         length(pos_Z)
%         AnkleR
        AnkleR = [AnkleR;[pos_X pos_Y pos_Z]];
        dAnkleR = [dAnkleR;[vec_X vec_Y vec_Z]];
        current_time = timeR(end);
    end
end
% Adding final state of Right
% when robot stop: both feet in double support phase
% tmpTime = (timeR(end):dt:WalkkingTime)';
% timeR = [timeR;tmpTime];
% AnkleR = [AnkleR;repmat(AnkleR(end,:),size(tmpTime))];
%%% It OK!
%%%----------------------------------

%% Create Ankle data for Left leg
timeL = [];
AnkleL = [];
dAnkleL = [];
current_time = 0;
for idx = 1:length(LFeet)-1
    if (LFeet(idx,1) == LFeet(idx+1,1))
        % Left Leg is in Stance phase
        timeL = [timeL;current_time+STPeriod];
        AnkleL = [AnkleL;repmat(LFeet(idx,:),size(STPeriod))];
        dAnkleL = [dAnkleL;repmat([0 0 0],size(STPeriod))];
        current_time = timeL(end);
    else
        % Left Leg is in Swing phase
        timeL = [timeL;current_time+SWPeriod];        
        [~,pos_X,vec_X] = UXA_Ankle_X(LFeet(idx,1),LFeet(idx+1,1),timeLFeet(idx),timeLFeet(idx+1),0.01);        
        [~,pos_Z,vec_Z] = UXA_Ankle_Z(stepHeight,timeLFeet(idx),timeLFeet(idx+1),0.01);
        pos_Y = repmat(LFeet(idx,2),size(SWPeriod));
        vec_Y = zeros(size(pos_Y));
        AnkleL = [AnkleL;[pos_X pos_Y pos_Z]];
        dAnkleL = [dAnkleL;[vec_X vec_Y vec_Z]];
        current_time = timeL(end);
    end
end
% Vi tri ban dau hai chan o vi tri double support
% Vi then trong khoang thoi gian 0 ->T_ds, chan trai dung yen
% Bo sung them khoang thoi gian do vao chan trai
% Xem them hinh anh human walking giat (search google)
LeftFeet_Init = repmat(LFeetStance(1,:),size(DSPeriod));
Vec_LeftFeet_Init = repmat([0 0 0],size(DSPeriod));
LeftTime_Init = DSPeriod;
AnkleL = [LeftFeet_Init;AnkleL];
dAnkleL = [Vec_LeftFeet_Init;dAnkleL];
timeL = [LeftTime_Init;LeftTime_Init(end)+timeL];
% Adding final state of Left Leg
% when robot stop: both feet in double support phase
% tmpTime = (timeL(end):dt:WalkkingTime)';
% timeL = [timeL;tmpTime];
% AnkleL = [AnkleL;repmat(AnkleL(end,:),size(tmpTime))];
%%% It OK!
%%%----------------------------------

% %% Remove duplicate value in time series of data
% NumData = length(timeL);
% DuplicateIdx = [];
% for idx = 1:NumData-1
%     if timeL(idx) == timeL(idx+1)
%         DuplicateIdx = [DuplicateIdx;idx];       
%     end
% end
% timeL(DuplicateIdx)=[];
% AnkleL(DuplicateIdx,:) = [];
% %---
% NumData = length(timeR);
% DuplicateIdx = [];
% for idx = 1:NumData-1
%     if timeR(idx) == timeR(idx+1)
%         DuplicateIdx = [DuplicateIdx;idx];       
%     end
% end
% timeR(DuplicateIdx)=[];
% AnkleR(DuplicateIdx,:) = [];
RAnkle = [timeR AnkleR];
LAnkle = [timeL AnkleL];

end

