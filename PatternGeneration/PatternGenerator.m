function [time,LAnkle,RAnkle,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef,ZMPout] = PatternGenerator( WalkingParameter,Zcom )
%PatternGenerator Summary of this function goes here
%   Detailed explanation goes here
% Global variable
global Tcycle DSrate SSrate SWrate STrate dt stepHeight
Tcycle = 1.4;
DSrate = 0.2; % Double support
SSrate = 0.8; % Single support
SWrate = 0.4; % Swing  phase
STrate = 0.6; % Stance phase
dt = 0.01;    % Sampling time of trajectory
stepHeight = 0.03;

HipWidth = 0.114;
% 
% FootStepDesign = [     0     walkstep walkstep walkstep      0;
%                     HipWidth HipWidth HipWidth HipWidth HipWidth];    
FootStepDesign = WalkingParameter;                

% ZMPDesign      = [     0     walkstep walkstep walkstep     0;
%                   HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];
ZMPDesign = WalkingParameter;
ZMPDesign(2,1) = ZMPDesign(2,1)/2;
ZMPDesign(2,end) = ZMPDesign(2,end)/2;

[LAnkle,RAnkle,dLAnkle,dRAnkle] = CalStanceFootPlace(FootStepDesign,[0 HipWidth/2],'Left');
ZMPRef = CalZMPRef(ZMPDesign,[0 0],'Left');

%%% Thoi gian tao quy dao ZMP, LeftFoot, RightFoot la khac nhau  vi vay can
%%% phai dong bo hoa thoi gia cua 3 quy dao duoc tao ra
time_end = [ZMPRef(end,1) LAnkle(end,1) RAnkle(end,1)];
[max_time,~] = max(time_end);
%display(time_end)
tmpTime = (RAnkle(end,1):dt:max_time)';
tmp_time = [RAnkle(:,1);tmpTime];
tmp_pos  = [RAnkle(:,2:4);repmat(RAnkle(end,2:4),size(tmpTime))];
dRAnkle  = [dRAnkle;repmat(dRAnkle(end,:),size(tmpTime))];
RAnkle = [tmp_time tmp_pos];
% LEFT 
tmpTime = (LAnkle(end,1):dt:max_time)';
tmp_time = [LAnkle(:,1);tmpTime];
tmp_pos  = [LAnkle(:,2:4);repmat(LAnkle(end,2:4),size(tmpTime))];
dLAnkle  = [dLAnkle;repmat(dLAnkle(end,:),size(tmpTime))];
LAnkle = [tmp_time tmp_pos];
% ZMP
tmpTime = (ZMPRef(end,1):dt:max_time)';
tmp_time = [ZMPRef(:,1);tmpTime];
tmp_pos_1 = [ZMPRef(:,2);repmat(ZMPRef(end,2),size(tmpTime))];
tmp_pos_2 = [ZMPRef(:,3);repmat(ZMPRef(end,3),size(tmpTime))];

ZMPRef = [tmp_time tmp_pos_1 tmp_pos_2];
%%% It OK!
%%%----------------------------------

%% Remove duplicate value in time series of data
NumData = length(ZMPRef(:,1));
DuplicateIdx = [];
for idx = 1:NumData-1
    if ZMPRef(idx,1) == ZMPRef(idx+1,1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end
ZMPRef(DuplicateIdx,:) = [];
%---
NumData = length(LAnkle);
DuplicateIdx = [];
for idx = 1:NumData-1
    if LAnkle(idx,1) == LAnkle(idx+1,1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end

LAnkle(DuplicateIdx,:) = [];
dLAnkle(DuplicateIdx,:) = [];
%----
NumData = length(RAnkle);
DuplicateIdx = [];
for idx = 1:NumData-1
    if RAnkle(idx,1) == RAnkle(idx+1,1)
        DuplicateIdx = [DuplicateIdx;idx];       
    end
end

RAnkle(DuplicateIdx,:) = [];
dRAnkle(DuplicateIdx,:) = [];
%%% Them thoi gian chuan bi buoc di = Tcycle/2
time = RAnkle(:,1);
time_prepare = (0:dt:Tcycle/2-dt)';
len_data = length(time_prepare);
time = [time_prepare;time+Tcycle/2];

tmp = repmat(RAnkle(1,:),[len_data,1]);
RAnkle = [tmp;RAnkle];
tmp = repmat(dRAnkle(1,:),[len_data,1]);
dRAnkle = [tmp;dRAnkle];

tmp = repmat(LAnkle(1,:),[len_data,1]);
LAnkle = [tmp;LAnkle];
tmp = repmat(dLAnkle(1,:),[len_data,1]);
dLAnkle = [tmp;dLAnkle];

tmp = repmat(ZMPRef(1,:),[len_data,1]);
ZMPRef = [tmp;ZMPRef];
% RAnkle
% LAnkle
% [CoM,dCoM,ZMPout] = CalCoMbasedCartTable( ZMPRef(:,2:3),dt,Zcom);
[CoM,dCoM,ZMPout] = calc_preview_control(ZMPRef(:,2:3),Zcom,time(end),1,dt);

RAnkle(:,1) = [];
LAnkle(:,1) = [];
ZMPRef(:,1) = [];
end

