function sam_bit = sendjointangle2uxa( jnt_deg)
%sendjointangle2uxa Send joint angle from simulation to uxa robot
%   We need to match between joint ID (in simulation) and sam ID ( uxa
%   robot)
% jnt_deg = [RJoints LJoints]
% port   : serial port of robot

%%% Output sam_bit
% sam_bit = [sam_0 sam_1 ..... sam_11];

SAM_CONFIG8BIT =  uxa_sam_config;
% SAM IDS ARE DIFFERENT JOINT ID (KINEMATIC PROBLEM)
% WE SHOULD LINK THOSE ID
Leg_joint_id = [ 2 3 4 5 6 7  8 9 10 11 12 13];
Leg_sam_id =   [11 9 7 5 3 1 10 8  6  4  2  0];

NumLegJoints = 12;
% index:        1 2 3 4 5 6 7 8 9  10 11 12
% JNT_ID:       2 3 4 5 6 7 8 9 10 11 12 13
% JointPos_deg: * * * * * * * *  *  *  *  *
% index = JNT_ID - 1;
JointPos_deg = jnt_deg;

% index:        1 2 3 4 5 6 7 8 9 10 11 12
% SAM_ID:       0 1 2 3 4 5 6 7 8  9 10 11
% JointPos_bit: * * * * * * * * *  *  *  *
% index = SAM_ID + 1;
JointPos_bit = 0*JointPos_deg;

bit2deg = 1.08; % Resolution of SAM in 8 bit mode 
[numofrows,~] = size(JointPos_bit(:, 1));

% Loop from Joint ID 0 to 11
for idx = 1:NumLegJoints
    JNT_ID = idx - 1;
    % SAM_ID of this JNT_ID
    sam_id = Leg_sam_id(idx);    
    
    for row = 1:numofrows        
        JointPos_bit(row,sam_id+1) = JointPos_deg(row,idx)*SAM_CONFIG8BIT(4,sam_id+1)/...
            bit2deg + SAM_CONFIG8BIT(2,sam_id+1);
        JointPos_bit(row,sam_id+1) = round(JointPos_bit(row,sam_id+1)+0.5);
    end
end
sam_bit = JointPos_bit;
% port = 0;

Torq = 1;
for n = 1:NumLegJoints
    uxa_set_jointAngle(Torq,n-1,sam_bit(1,n));
end

%%% ------
end

function CONFIG8BIT = uxa_sam_config()
%%% CREATE UXA SAM CONFIG 
% SAM_ID        0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21 22 23
LowerLimit = [254 254   1 254   1 254 254   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1  1];
StandupPos = [127 128 128 127  41 212 129 127 128 128 127 128 127 127 127 127 127 127 127 127 127 127 127];
UpperLimit = [  1   1 254   1 254   1   1 254 254 254 254 254 254 254 254 254 254 254 254 254 254 254 254];
Dirrect = [-1 -1 1 -1 1 -1 -1 1 -1 -1 1 1 1 1 1 1 1 1 1 1 1 1 1];
CONFIG8BIT = [LowerLimit;StandupPos;UpperLimit;Dirrect];
end

