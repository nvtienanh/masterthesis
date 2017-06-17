clear all
close all
clc
addpath('First_4Step')
addpath('Matfile');

FootWidth = 71;
FootHeight = 200;

walkstep = 0.05;
HipWidth = 0.114;
    
FootStepDesign = [     0     walkstep walkstep walkstep      0;
                    HipWidth HipWidth HipWidth HipWidth HipWidth];                

ZMPDesign      = [     0     walkstep walkstep walkstep     0;
                  HipWidth/2 HipWidth HipWidth HipWidth HipWidth/2];

              
[time,LAnkle,RAnkle,dLAnkle,dRAnkle,CoM,dCoM,ZMPRef,ZMPout] = PatternGenerator(FootStepDesign,0.45);            

% save 'Matfile/time.mat' time
% save 'Matfile/LAnkle.mat' LAnkle
% save 'Matfile/RAnkle.mat' RAnkle
% save 'Matfile/CoM.mat' CoM
% save 'Matfile/dCoM.mat' dCoM
% save 'Matfile/ZMPRef.mat' ZMPRef
% save 'Matfile/ZMPout.mat' ZMPout
% 
% load('Matfile/time.mat');
% load('Matfile/LAnkle.mat');
% load('Matfile/RAnkle.mat');
% load('Matfile/CoM.mat');
% load('Matfile/dCoM.mat');
% load('Matfile/ZMPRef.mat');
% load('Matfile/ZMPout.mat');

figure
subplot(2,1,1)
hold on
plot(time,CoM(:,1)/1000,'-r');
plot(time,ZMPRef(:,1)/1000,'--r');
plot(time,ZMPout(:,1)/1000,'-b');
grid on
subplot(2,1,2)
hold on
plot(time,CoM(:,2)/1000,'-r');
plot(time,ZMPRef(:,2)/1000,'--r');
plot(time,ZMPout(:,2)/1000,'-b');
grid on
hold off

load('LeftData.mat');
load('RightData.mat');
load ('ZMP_m.mat');
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

ZMPL = dtrend(ZMPL,1);
ZMPR = dtrend(ZMPR,1);

figure
hold on
plot(time,ZMPL,'-r');
plot(time,ZMPR,'-b');
legend('Right Foot','Left Foot')
grid on
xlabel('time');
ylabel('Magnitude');


% position in world coordinate
ZMPL_x = -(HipWidth/2*1000+(FootWidth+0)/2-ZMPL_x);
ZMPR_x = (HipWidth/2*1000-(FootWidth-50)/2+ZMPR_x);


% offset value to match value between simulation and experiment
offset = 85;
idxdata = offset + [13 73 85 145 157 217 229 289 301]';

ZMPX = zeros(size(time));
ZMPY = zeros(size(time));


%%% Match experiment and simulation time
oslen = 10;
tmp = repmat(RAnkle(1,:),[oslen,1]);
RAnkle = [tmp;RAnkle];
tmp = repmat(LAnkle(1,:),[oslen,1]);
LAnkle = [tmp;LAnkle];
tmp = repmat(ZMPRef(1,:),[oslen,1]);
ZMPRef = [tmp;ZMPRef];
tmp = repmat(ZMP_m(1,:),[oslen,1]);
ZMP_m = [tmp;ZMP_m];
tmp = repmat(ZMPout(1,:),[oslen,1]);
ZMPout = [tmp;ZMPout];
tmp = repmat(CoM(1,:),[oslen,1]);
CoM = [tmp;CoM];
tmp = repmat(dCoM(1,:),[oslen,1]);
dCoM = [tmp;dCoM];

%%% Match experiment and simulation time
tmp = repmat(RAnkle(end,:),[length(ZMPX)-length(RAnkle),1]);
RAnkle = [RAnkle;tmp];
tmp = repmat(LAnkle(end,:),[length(ZMPX)-length(LAnkle),1]);
LAnkle = [LAnkle;tmp];
tmp = repmat(ZMPRef(end,:),[length(ZMPX)-length(ZMPRef),1]);
ZMPRef = [ZMPRef;tmp];

tmp = repmat(ZMP_m(end,:),[length(ZMPX)-length(ZMP_m),1]);
ZMP_m = [ZMP_m;tmp];

tmp = repmat(ZMPout(end,:),[length(ZMPX)-length(ZMPout),1]);
ZMPout = [ZMPout;tmp];
tmp = repmat(CoM(end,:),[length(ZMPX)-length(CoM),1]);
CoM = [CoM;tmp];
tmp = repmat(dCoM(end,:),[length(ZMPX)-length(dCoM),1]);
dCoM = [dCoM;tmp];

cnt = 1;
% Double support
for idx = 1:idxdata(cnt)-1
    ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
    ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
end
% Single support right leg
cnt = cnt + 1;
tmpidx1 =  idxdata(cnt-1);
tmpidx2 = idxdata(cnt)-1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
%     ZMPX(idx) = ZMPR_x(idx);
    ZMPX(idx) = ZMPL_x(idx);
    ZMPY(idx) = ZMPR_y(idx) + RAnkle(idx,1)*1000;
end
figure
plot(ZMPL_x(tmpidx1:tmpidx2),ZMPL_y(tmpidx1:tmpidx2));

% Double support,
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
    ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
end
% Single support Left leg
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx) = ZMPR_x(idx);
%     ZMPX(idx) = ZMPL_x(idx);
    ZMPY(idx) = ZMPL_y(idx) + LAnkle(idx,1)*1000;
end
% Double support
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
    ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
end
% Single support Right leg 
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx)= ZMPL_x(idx);
%     ZMPX(idx)= ZMPR_x(idx);
    ZMPY(idx) = ZMPR_y(idx) + RAnkle(idx,1)*1000;
end
% Double support
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
    ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
end
% Single support Left leg
cnt = cnt + 1;
for idx = idxdata(cnt-1):idxdata(cnt)-1
    ZMPX(idx) = ZMPR_x(idx);
%     ZMPX(idx) = ZMPL_x(idx);
    ZMPY(idx) = ZMPL_y(idx) + LAnkle(idx,1)*1000;
end
% Double support
cnt = cnt + 1;
for idx = idxdata(cnt-1):length(time)
    ZMPX(idx) = ZMPL_x(idx) + ZMPR_x(idx);
    ZMPY(idx) = (ZMPL_y(idx) + LAnkle(idx,1)*1000)/2 + (ZMPR_y(idx) + RAnkle(idx,1)*1000)/2;
end

% ZMP in word coordinate: y direction
% ZMPX = dtrend(ZMPX,1);
figure
subplot(2,1,1)
hold on
plot(time, ZMPX,'-r');
plot(time,ZMPRef(:,2)*1000,'--r');
plot(time,ZMP_m(:,2)*1000,'-b');
plot(time,ZMPout(:,2)*1000,'--b');
grid on
subplot(2,1,2)
hold on
plot(time, ZMPY - ZMPY(1),'-r');
plot(time,ZMPRef(:,1)*1000,'--r');
plot(time,ZMP_m(:,1)*1000,'-b');
plot(time,ZMPout(:,1)*1000,'--b');
grid on


figure
subplot(2,1,1)
hold on
plot(time,ZMPRef(:,2)*1000,'--b','LineWidth',1);
plot(time,ZMPout(:,2)*1000,'-r','LineWidth',1);
plot(time,CoM(:,2)*1000,'-g','LineWidth',1);
xlabel('time');
ylabel('y (mm)');
legend('ZMP_{ref}','ZMP_{out}','CoM');
ylim([-60 60]);
grid on
subplot(2,1,2)
hold on
plot(time,ZMPRef(:,1)*1000,'--b','LineWidth',1);
plot(time,ZMPout(:,1)*1000,'-r','LineWidth',1);
plot(time,CoM(:,1)*1000,'-g','LineWidth',1);
xlabel('time');
ylabel('x (mm)');
legend('ZMP_{ref}','ZMP_{out}','CoM');
ylim([-10 160]);
grid on

figure
subplot(2,1,1)
hold on
% plot(time,CoM(:,1)*1000,'-g','LineWidth',1);
plot(time,LAnkle(:,1)*1000,'-m','LineWidth',1);
plot(time,RAnkle(:,1)*1000,'-b','LineWidth',1);
legend('LeftAnkle','RightAnkle');
xlabel('time');
ylabel('x(mm)');
grid on
ylim([-10 160]);
subplot(2,1,2)
hold on
plot(time,LAnkle(:,3)*1000+65,'-m','LineWidth',1);
plot(time,RAnkle(:,3)*1000+65,'-b','LineWidth',1);
legend('LeftAnkle','RightAnkle');
xlabel('time');
ylabel('z(mm)');
grid on
ylim([60 100]);

zmpx = ZMPY - ZMPY(1);
zmpy = ZMPX;
zmpx(end) = [];
zmpy(end) = [];
ZMPRef(end,:) =[];