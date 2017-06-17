%%% Experiment data
clear all
close all
clc

load('LeftData.mat');
load('RightData.mat');

t_begin=24000;
t_end=25000;

t_begin_1=23000;
t_end_1=24500;

 
timeL = LeftData{7}(t_begin:t_end,1);
timeR = RightData{7}(t_begin_1:t_end_1,1);
ZMPL = LeftData{7}(t_begin:t_end,2);
ZMPR = RightData{7}(t_begin_1:t_end_1,2);

ZMPL_x = LeftData{5}(t_begin:t_end,2);
ZMPR_x = RightData{5}(t_begin_1:t_end_1,2);
ZMPL_y = LeftData{6}(t_begin:t_end,2);
ZMPR_y = RightData{6}(t_begin_1:t_end_1,2);

idxL_1 = 189;
idxL_2 = 689;

idxR_1 = 471;
idxR_2 = 916;
ZMPL = ZMPL(idxL_1:idxL_2);
ZMPL_x = ZMPL_x(idxL_1:idxL_2);
ZMPL_y = ZMPL_y(idxL_1:idxL_2);

timeL = timeL(idxL_1:idxL_2);
timeL = timeL - timeL(1);

ZMPR = ZMPR(idxR_1:idxR_2);
ZMPR_x = ZMPR_x(idxR_1:idxR_2);
ZMPR_y = ZMPR_y(idxR_1:idxR_2);

timeR = timeR(idxR_1:idxR_2);
timeR = timeR - timeR(1);

[~, ZMPL] = even_sample(timeL, ZMPL, 1/0.01);
[~, ZMPL_x] = even_sample(timeL, ZMPL_x, 1/0.01);
[~, ZMPL_y] = even_sample(timeL, ZMPL_y, 1/0.01);

[~, ZMPR] = even_sample(timeR, ZMPR, 1/0.01);
[~, ZMPR_x] = even_sample(timeR, ZMPR_x, 1/0.01);
[~, ZMPR_y] = even_sample(timeR, ZMPR_y, 1/0.01);

time = zeros(length(ZMPL),1);

for idx = 2:length(ZMPL)
    time(idx) = time(idx-1)+ 0.01;
end


figure
hold on
plot(time,ZMPL,'-r');
plot(time,ZMPR,'-b');
grid on

% Position in sensor coordinate
ZMPL = smooth(ZMPL,0.02,'loess');
ZMPR = smooth(ZMPR,0.02,'loess');
ZMPL_x = smooth(ZMPL_x,0.02,'loess');
ZMPR_x = smooth(ZMPR_x,0.02,'loess');
ZMPL_y = smooth(ZMPL_y,0.02,'loess');
ZMPR_y = smooth(ZMPR_y,0.02,'loess');
% Plot after smooth
figure
hold on
plot(time,ZMPL,'-k');
plot(time,ZMPR,'--k');
grid on
legend('Left','Right');

figure
subplot(2,1,1)
hold on
plot(time,ZMPL_x,'-r');
plot(time,ZMPR_x,'-b');
grid on
subplot(2,1,2)
hold on
plot(time,ZMPL_y,'-r');
plot(time,ZMPR_y,'-b');
grid on