function ZMPRefData = CalZMPRef(ZMPDesignTable,ZMP_init,SwingFirst)
%CalZMPRef Function calculate ZMP reference from Walking table
%   Detailed explanation goes here
global Tcycle DSrate SSrate dt
zmp_x = ZMPDesignTable(1,:);
zmp_y = ZMPDesignTable(2,:);

[~,NumStep] = size(ZMPDesignTable);

if strcmp(SwingFirst,'Left')
    LeftStanceFirst = 0;
else
    LeftStanceFirst = 1;
end

ZMPRef = zeros(NumStep+1,2); % Stance Foot placement Desired
ZMPRef(1,1) = ZMP_init(1);
ZMPRef(1,2) = ZMP_init(2);

for n = 1:NumStep 
    % Equation 4.50    
    ZMPRef(n+1,1)     = ZMPRef(n,1) + zmp_x(n);
    if LeftStanceFirst        
        ZMPRef(n+1,2) = ZMPRef(n,2) - (-1)^(n)*zmp_y(n);
    else        
        ZMPRef(n+1,2) = ZMPRef(n,2) + (-1)^(n)*zmp_y(n);  
    end    
end

% Init state before calculate
T_ss = Tcycle/2*SSrate; % Single support
T_ds = Tcycle/2*DSrate; % Double support
DSPeriod = 0:dt:T_ds;DSPeriod = DSPeriod'; % Double support
NumPointDS = length(DSPeriod);
SSPeriod = 0:dt:T_ss;SSPeriod = SSPeriod'; % Single support
NumPointSS = length(SSPeriod);

time = [];

%%% Support state
% SupportState = 1 : Double support phase
% SupportState = 0 : Single support phase
% SupportState = 1;
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

% %% Remove duplicate value in time series
% NumData = length(time);
% DuplicateIdx = [];
% for idx = 1:NumData-1
%     if time(idx) == time(idx+1)
%         DuplicateIdx = [DuplicateIdx;idx];       
%     end
% end
% time(DuplicateIdx)=[];
% ZMP_x(DuplicateIdx) = [];
% ZMP_y(DuplicateIdx) = [];
%%% ----------------------------------------
ZMPRefData = [time ZMP_x ZMP_y];
end

