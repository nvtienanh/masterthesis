%% Record Joint position
clc
% open_uxa_serial;
% uic_motion_cmd('pc_control');
% pause
% uic_motion_cmd('standup');
% pause
%%% ZeroPosition
% ID  : 0   1   2   3   4  5   6  7   8   9   10  11  12 13  14  15  16  17  18  19  20  21  22  23  24
% Zero: 130 129 129 131 38 214 85 172 128 132 131 130 87 169 134 125 128 126 215 43  127 127 127 130 125
% ZeroPos = [130 129 129 131 38 214 85 172 128 132 131 130 87 169 134 125 ...
%             128 126 215 43 127 127 127 130 125]';
% Command Packet
% StandupPos = zeros(12,1);
% for samID=1:1:12
%     [Load,Pos] = uxa_get_jointAngle(samID-1);
%     StandupPos(samID) = Pos;    
% end
%%% LOAD JOINT VALUE
load 'SimData2UXA/RJoints.mat'
load 'SimData2UXA/LJoints.mat'

RJoints = rad2deg(RJoints);
LJoints = rad2deg(LJoints);
uxa_joints = zeros(1,12);

uxa_joints(11:-2:1) = LJoints(1,:);% Left Joints
uxa_joints(12:-2:2) = RJoints(1,:);% Right Joints
sam_config8bit;

% for idx = 1:12
%    samID = idx-1;
%    if idx == 7 || idx == 8
%            Numbit = round((uxa_joints(idx))/0.05);
%        else
%            Numbit = round(uxa_joints(idx)/0.05);           
%    end
%    data2send(idx) = SAM_CONFIG10BIT(2,idx)+Numbit*SAM_CONFIG10BIT(4,idx);
%    if data2send(idx) > 3600
%        data2send(idx) = 3600;
%    elseif data2send(idx) < 500
%        data2send(idx) = 500;
%    end
%    tic;
%    uxa_set_jointAngle10bit(samID,data2send(idx));
%    toc
% end

% pause
JointsPos = zeros(length(RJoints),12);
JointsLoad = zeros(length(RJoints),12);
JointsRef = zeros(length(RJoints),12);
for idxx=1:length(RJoints)
    uxa_joints(11:-2:1) = LJoints(idxx,:);% Left Joints
    uxa_joints(12:-2:2) = RJoints(idxx,:);% Right Joints
    
    for idx = 1:12
       samID = idx-1;
       if idx == 7 || idx == 8
           Numbit = round((uxa_joints(idx))/1.08);
       else
           Numbit = round(uxa_joints(idx)/1.08);           
       end
       
       JointsRef(idxx,idx) = SAM_CONFIG8BIT(2,idx)+Numbit*SAM_CONFIG8BIT(4,idx);
       if JointsRef(idxx,idx) > 254
           JointsRef(idxx,idx) = 254;
       elseif JointsRef(idxx,idx) < 2
           JointsRef(idxx,idx) = 1;
       end
%        uxa_set_jointAngle(2,samID,JointsRef(idxx,idx));
       
    end
    uxa_Sync_jointAngle(2,11,JointsRef(idxx,:));
%     display(JointsRef(idxx,:));
    
    
if idxx==1
    pause;
else
    pause;
end
% %     Read Joints Pos
%     for idx = 1:12
%         samID = idx-1;
%         [Load,Pos] = uxa_get_jointAngle(samID);        
%         display([Load Pos]);
%         JointsPos(idxx,idx) = Pos;
%         JointsLoad(idxx,idx) = Load;
%         pause(0.005);
%     end
end

%% Plot graph




    
