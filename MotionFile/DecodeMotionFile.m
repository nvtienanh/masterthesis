%%% Read data from motion file of robot builder
% DATA AFTER 7 ':" SYMBOL
% 1:13:0000000000000:XXXXXXXXXX:XXXXXXXXXXXXXXXXXXXX:12:M04_WF_SHORT:
% 1:13:  SerialCode :  Author  :     EmailAddress   :12: MotionName :
%
% close all
% clear
% clc
text = fileread('M08_WF_4STEP.rbm');
LenData = length(text); % Length of data
% Calculate Number of ':' symbol
NumofDDot=0;
for idx=1:LenData
    if text(idx)==':'
        NumofDDot = NumofDDot+1;
    end    
end

% Pre-allocal data
DDot = zeros(NumofDDot,1);
DDotIDX = 0;
for idx=1:LenData
    if text(idx)==':'
        DDotIDX = DDotIDX+1;
        DDot(DDotIDX) = idx;
    end    
end

%% Proceess Data
SerialCode = text(DDot(2)+1:DDot(3)-1);
Author = text(DDot(3)+1:DDot(4)-1);
Email = text(DDot(4)+1:DDot(5)-1);
MotionName = text(DDot(6)+1:DDot(7)-1);

NumOfScene = str2double(text(DDot(9)+1:DDot(10)-1));
DefaultTorq = str2double(text(DDot(10)+1:DDot(11)-1));
NumOfServo = str2double(text(DDot(11)+1:DDot(12)-1));
MotionData = cell(1,NumOfScene);

ZerosPos = zeros(NumOfServo,1); % Zeros Pos in bit Mode (8bit)
PGain = zeros(NumOfServo,1); % PGain (8bit)
DGain = zeros(NumOfServo,1); % DGain (8bit)
IGain = zeros(NumOfServo,1); % IGain (8bit)
% Read Initial State of Motion Data
Bidx = 13;
EndDDot = Bidx + NumOfServo*6;

for SAMID = 0 : NumOfServo - 1
    SAMIDX = SAMID + 1;
    % ID = text(DDot(Bidx)+1:DDot(Bidx+1)-1);
    Bidx = Bidx + 1;    
    PGain(SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
    Bidx = Bidx + 1;
    DGain(SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
    Bidx = Bidx + 1;
    IGain(SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
    Bidx = Bidx + 1;
    % Data = 0
    Bidx = Bidx + 1;
    ZerosPos(SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1)); 
    Bidx = Bidx + 1;
end

% Read Scence Data
ScenceData = zeros (NumOfScene,3+6*NumOfServo);
NumofFrame = zeros(NumOfScene,1);
TrTime = NumofFrame;
SPos = zeros(NumOfScene,NumOfServo);
DPos = SPos;
Torq = SPos;

Bidx = Bidx + 2;
for idx = 1:NumOfScene
    % Scene Name
%     text(DDot(Bidx)+1:DDot(Bidx+1)-1)
    Bidx = Bidx + 1;
    NumofFrame(idx) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
    Bidx = Bidx + 1;
    TrTime(idx) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
    Bidx = Bidx + 1;
    % Don't use this data (0)
    Bidx = Bidx + 1;
    % Don't use this data (SAM ID)   
    for SAMID = 0 : NumOfServo - 1
        SAMIDX = SAMID + 1;
        Bidx = Bidx + 1;
        SPos(idx,SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
        Bidx = Bidx + 1;
        DPos(idx,SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
        Bidx = Bidx + 1;
        Torq(idx,SAMIDX) = str2double(text(DDot(Bidx)+1:DDot(Bidx+1)-1));
        Bidx = Bidx + 1;
        % This data is Zeros
        Bidx = Bidx + 1;
        % This data is Zeros
        Bidx = Bidx + 1;
        % This data is Zeros        
    end
    Bidx = Bidx + 1;
end

TotalFrames = sum(NumofFrame);
Times = zeros(TotalFrames+1,1);
Bidx = 1;
% Calcualte Sampling Time
for idx = 1:NumOfScene
    NumFrame = NumofFrame(idx);    
    % Calculate Sampling Time 
    Times(Bidx:Bidx+NumFrame) = linspace(Times(Bidx),Times(Bidx)+TrTime(idx),NumFrame+1);
    Bidx = Bidx + NumFrame;
end

% Calculate Sampling Data
SamPos = zeros(TotalFrames+1,NumOfServo);
SamPos(1,:) = SPos(1,:); % Initial Postion
Bidx = 1;
for idx = 1:NumOfScene
    NumFrame = NumofFrame(idx);
    % Calculate Sampling Data for each SAM ID
    for Samidx=1:NumOfServo
        SamPos(Bidx:Bidx+NumFrame,Samidx) = linspace(SPos(idx,Samidx),DPos(idx,Samidx),NumFrame+1);
        
    end
    MotionData{idx} = SamPos(Bidx:Bidx+NumFrame,:);
    Bidx = Bidx + NumFrame;
end


% % Plot data
% NumSamUXA = 11;
% for idx = 1:NumSamUXA
%     figure(idx)
%     hold on
%     plot(Times,SamPos(:,idx),'-r');
%     grid on    
% end
% 



