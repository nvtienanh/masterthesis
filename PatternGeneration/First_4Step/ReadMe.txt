/** Data layout*/
Hardwawe: ZMPV1.0
Date: 22/2/2016
Name: PNT

for each variable: @LeftData and @RightData
[time,channel_1],[time,channel_2],[time,channel_3],[time,channel_4],[time,channel_5],[time,channel_6],[time,channel_7]


Channel_1 = sensor(1) of LeftData ; sensor(5) of RightData
Channel_2 = sensor(2) of LeftData ; sensor(6) of RightData
Channel_3 = sensor(3) of LeftData ; sensor(7) of RightData
Channel_4 = sensor(4) of LeftData ; sensor(8) of RightData

Sensor(i) is ADC value of each sensor with unit: 400ADC10bit unit=5000gf (gram force)

Channel_7=zmp_amplitude = Channel_1 + Channel_2+ Channel_3 + Channel_4;
Channel_5= zmp_position_x = (Channel_1 * ZMP_POSITION_0_X + Channel_2  * ZMP_POSITION_1_X + Channel_3  * ZMP_POSITION_2_X + Channel_4 * ZMP_POSITION_3_X) / zmp_amplitude;
Channel_6=zmp_position_y = (Channel_1 * ZMP_POSITION_0_Y + Channel_2 * ZMP_POSITION_1_Y + Channel_3 * ZMP_POSITION_2_Y + Channel_4* ZMP_POSITION_3_Y) / zmp_amplitude;

Event Timeline: Period when a 4STEP motion happen
Time =
240-250s
270-280s
290-300s
310-315s
328-332s
~346s



%========= Sensor Position =========
%              Front view 
%   -o-     -o-         -o-     -o-
%   (4)     (3)         (8)     (7) 
% 
%       Left                Right
% 
%   -o-     -o-         -o-     -o-
%   (1)     (2)         (5)     (6)
%y
%^
%|
%|--->x

//========define for the right leg======
#define ZMP_POSITION_0_X 0
#define ZMP_POSITION_0_Y 0

#define ZMP_POSITION_1_X 71
#define ZMP_POSITION_1_Y 0

#define ZMP_POSITION_2_X 64
#define ZMP_POSITION_2_Y 200

#define ZMP_POSITION_3_X -9.5
#define ZMP_POSITION_3_Y 200

//========define for the left leg=========
#define ZMP_POSITION_0_X 0
#define ZMP_POSITION_0_Y 0

#define ZMP_POSITION_1_X 71
#define ZMP_POSITION_1_Y 0

#define ZMP_POSITION_2_X 80.5
#define ZMP_POSITION_2_Y 200

#define ZMP_POSITION_3_X 7
#define ZMP_POSITION_3_Y 200