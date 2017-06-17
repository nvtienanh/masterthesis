%%% Experiment data
clear all
close all
clc

load('LeftData.mat');
load('RightData.mat');

% Event Timeline: Period when a 4STEP motion happen
% Time =
% 240-250s
% 270-280s
% 290-300s
% 310-315s
% 328-332s
% ~346s

t_begin_l=24000;
t_end_l=25000;

t_begin_r=23000;
t_end_r=24500;

 
timeL = LeftData{7}(:,1);
timeR = RightData{7}(:,1);
ZMPL = LeftData{7}(:,2);
ZMPR = RightData{7}(:,2);

figure
hold on
plot(timeL,ZMPL,'-r');
plot(timeR,ZMPR,'-b');
grid on
hold off

t_L = [20490 20975;
       24190 24690;
       26787 27285;
       29387 29887;
       30685 31189;
       32389 32988;
       33780 34285];
   
t_R = [20490 20975;
       24190 24690;
       26787 27285;
       29387 29887;
       30685 31189;
       32389 32987;
       33780 34285];
offset = [-676 -718;% +325 442
       -719 -775;
       -775 -776;
       -789 -823;
       -824 -850;
       -851 -862;
       -862 -865];
t_R = t_R + offset;   

%%% Plot all sample data
[row,~]= size(t_L);
dt = 0.01;
for idx = 1:1   
    timeL = LeftData{7}(t_L(idx,1):t_L(idx,2),1);
    timeL = timeL - timeL(1);
    timeR = RightData{7}(t_R(idx,1):t_R(idx,2),1);
    timeR = timeR - timeR(1);

    ZMPL = LeftData{7}(t_L(idx,1):t_L(idx,2),2);
    ZMPR = RightData{7}(t_R(idx,1):t_R(idx,2),2);
    ZMPL_x = LeftData{5}(t_L(idx,1):t_L(idx,2),2);
    ZMPR_x = RightData{5}(t_R(idx,1):t_R(idx,2),2);
    ZMPL_y = LeftData{6}(t_L(idx,1):t_L(idx,2),2);
    ZMPR_y = RightData{6}(t_R(idx,1):t_R(idx,2),2);
    
    %%% dong bo thoi gian
    [~, ZMPL] = even_sample(timeL, ZMPL, 1/dt);
    [~, ZMPL_x] = even_sample(timeL, ZMPL_x, 1/dt);
    [~, ZMPL_y] = even_sample(timeL, ZMPL_y, 1/dt);
% 
    [~, ZMPR] = even_sample(timeR, ZMPR, 1/dt);
    [~, ZMPR_x] = even_sample(timeR, ZMPR_x, 1/dt);
    [~, ZMPR_y] = even_sample(timeR, ZMPR_y, 1/dt);
% 
    time = zeros(length(ZMPL),1);

    for n = 2:length(ZMPL)
        time(n) = time(n-1)+ dt;
    end
%     ZMPL = dtrend(ZMPL,1);
%     ZMPR = dtrend(ZMPR,1);
    figure
    hold on
    plot(time,ZMPL,'--k');
    plot(time,ZMPR,'-k');
    legend('Right Foot', 'Left Foot');
    grid on
    hold off
end