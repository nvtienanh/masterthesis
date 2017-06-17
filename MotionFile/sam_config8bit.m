%%% CREATE UXA SAM CONFIG 
% SAM_ID        0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21 22 23
LowerLimit = [254 254   1 254   1 254 254   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1  1];
StandupPos = [127 128 128 127  41 212 129 127 128 128 127 128 127 127 127 127 127 127 127 127 127 127 127];
UpperLimit = [  1   1 254   1 254   1   1 254 254 254 254 254 254 254 254 254 254 254 254 254 254 254 254];
Dirrect = [-1 -1 1 -1 1 -1 -1 1 -1 -1 1 1 1 1 1 1 1 1 1 1 1 1 1];
SAM_CONFIG8BIT = [LowerLimit;StandupPos;UpperLimit;Dirrect];


% LowerLimit = [3600 3600 500 3600 500 3600 3600 500 500 500 500 500];
% StandupPos = [2051 2044 2046 2065 853 3153 1479 2593 2025 2082 2068 2058];
% UpperLimit = [500 500 3600 500 3600 500 500 3600 3600 3600 3600 3600];
% Dirrect = [-1 -1 1 -1 1 -1 -1 1 1 1 1 1];
% SAM_CONFIG10BIT = [LowerLimit;StandupPos;UpperLimit;Dirrect];

