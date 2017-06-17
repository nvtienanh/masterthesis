clear all
close all
clc
% Load joint data from simulation
load LJoints.mat % unit deg
load RJoints.mat

sam_config8bit;
% SAM IDS ARE DIFFERENT JOINT ID (KINEMATIC PROBLEM)
% WE SHOULD LINK THOSE ID
Leg_joint_id = [ 2 3 4 5 6 7  8 9 10 11 12 13];
Leg_sam_id =   [11 9 7 5 3 1 10 8  6  4  2  0];

NumLegJoints = 12;
% index:        1 2 3 4 5 6 7 8 9  10 11 12
% JNT_ID:       2 3 4 5 6 7 8 9 10 11 12 13
% JointPos_deg: * * * * * * * *  *  *  *  *
% index = JNT_ID - 1;
JointPos_deg = [RJoints LJoints];

% index:        1 2 3 4 5 6 7 8 9 10 11 12
% SAM_ID:       0 1 2 3 4 5 6 7 8  9 10 11
% JointPos_bit: * * * * * * * * *  *  *  *
% index = SAM_ID + 1;
JointPos_bit = 0*JointPos_deg;

bit2deg = 1.08; % Resolution of SAM in 8 bit mode 
[numofrows,~] = size(JointPos_bit(:, 1));

% Loop from Joint ID 0 to 11
for idx = 1:12
    JNT_ID = idx - 1;
    % SAM_ID of this JNT_ID
    sam_id = Leg_sam_id(idx);    
    
    for row = 1:numofrows        
        JointPos_bit(row,sam_id+1) = JointPos_deg(row,idx)*SAM_CONFIG8BIT(4,sam_id+1)/...
            bit2deg + SAM_CONFIG8BIT(2,sam_id+1);
        JointPos_bit(row,sam_id+1) = round(JointPos_bit(row,sam_id+1)+0.5);
    end
end
%%% ------

figure
hold on
plot(JointPos_bit(:,1))
plot(JointPos_bit(:,2))
plot(JointPos_bit(:,3))
plot(JointPos_bit(:,4))
plot(JointPos_bit(:,5))
plot(JointPos_bit(:,6))
grid on
title('SAM ID');
% legend('SAM_0','SAM_1','SAM_2','SAM_3','SAM_4','SAM_5');



