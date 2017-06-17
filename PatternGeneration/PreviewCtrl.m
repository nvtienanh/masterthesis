ZCoM = 500;% mm
dt = 0.001; % s
%% File: BipedPattern.m
% Title: Function creat reference trajectory for biped walking
% Author: Nguyen Van Tien Anh
% Created Date: 26/02/2016
% INPUT: 
%        + ZCoM: Height of CoM (Center of Mass)
%        + dt  : Sampling time of reference
% OUTPUT:
%        + times: Times walking
%        + Biped CoM: Desired CoM position
%        + LeftFoot : Desired Left Foot position
%        + RightFoot: Desired Right Foot position
%        + sFootX   : Desired ZMP in X direction
%        + sFootY   : Desired ZMP in Y direction
% All code based on book: Introduction to humanoid robotics
%%------------------------------------------------------------------------%

%% Walking cycle
Tds = 0.2;  % Double support period
Tss = 0.8;  % Single support period
g = 9810;   % 
T_c = sqrt(ZCoM/g);
% Walking parameter
walkstep = 200;
s_x = [0 walkstep/2 walkstep walkstep walkstep walkstep walkstep walkstep...
        walkstep walkstep/2 walkstep/2 0];
s_y = [100 100 100 100 100 100 100 100 100 100 100 100];
NumStep = length(s_x)-1;
% s = [s_x' s_y'];

poly4th = @(coeff,x) coeff(1)*x.^4+coeff(2)*x.^3+coeff(3)*x.^2+...
        coeff(4)*x+coeff(5);
dpoly4th = @(coeff,x) 4*coeff(1)*x.^3+3*coeff(2)*x.^2+2*coeff(3)*x+...
            coeff(4);
ddpoly4th = @(coeff,x) 12*coeff(1)*x.^2+6*coeff(2)*x+2*coeff(3);       

%% Step 1
% Initial CoM
x_init = 0;
y_init = 0;
dx_init = 0;
dy_init = 0;
FootInitial; % Foot parameter

%%
tmp = 0:dt:Tss;
tds = 0:dt:Tds;

T=[];
X_CoM=[];
dX_CoM=[];
ddX_CoM=[];
Y_CoM=[];
dY_CoM=[];
ddY_CoM=[];
RFootXY = [];
LFootXY = [];
px = zeros(NumStep,1);
py = px;
px_mod = px;
py_mod = py;

x_ = zeros(NumStep,1);
y_ = x_;
dx_ = x_;
dy_ = x_;

x_d = x_;
y_d = x_;
dy_d = x_;
dx_d = x_;

% Stance Foot
sFootX=[];
sFootY=[];

C = cosh((Tss+Tds/2)/T_c);
S = sinh((Tss+Tds/2)/T_c);
a = 10;
b = 1;    
D = a*(C-1)^2+b*(S/T_c)^2;

%% Initital Foot Placement
px_0 = 0;
py_0 = -0.1;
pdot_x = px_0;
pdot_y = py_0;
x_0 = s_x(1)/2;
y_0 = s_y(1)/2;
dx_0 = (C+1)/(T_c*S)*x_0;
dy_0 = (C-1)/(T_c*S)*y_0;

x_d_0 = px_0+x_0;
dx_d_0 = dx_0;        
y_d_0 = py_0+y_0;
dy_d_0 = dy_0;

px_mod_0 = -a*(C-1)/D*(x_d_0-C*x_init-T_c*S*dx_init)- ...
                b*S/T_c/D*(dx_d_0-S/T_c*x_init-C*dx_init);      
py_mod_0 = -a*(C-1)/D*(y_d_0-C*y_init-T_c*S*dy_init)- ...
                b*S/T_c/D*(dy_d_0-S/T_c*y_init-C*dy_init);

t = tmp;
x = (x_init-px_mod_0)*cosh(t/T_c)+T_c*dx_init*sinh(t/T_c)+px_mod_0;
dx = (x_init-px_mod_0)/T_c*sinh(t/T_c)+dx_init*cosh(t/T_c);
ddx = (x_init-px_mod_0)/T_c^2*cosh(t/T_c)+dx_init/T_c*sinh(t/T_c);
y =(y_init-py_mod_0)*cosh(t/T_c)+T_c*dy_init*sinh(t/T_c)+py_mod_0;
dy =(y_init-py_mod_0)/T_c*sinh(t/T_c)+dy_init*cosh(t/T_c);  
ddy = (y_init-py_mod_0)/T_c^2*cosh(t/T_c)+dy_init/T_c*sinh(t/T_c);

% Begining Double support phase
% xds_begin = x(end);
% yds_begin = y(end);
% dxds_begin = dx(end);
% dyds_begin = dy(end);
% ddxds_begin = ddx(end);
% ddyds_begin = ddy(end);
x_init = x(end);
y_init = y(end);
dx_init = dx(end);
dy_init = dy(end);
      
    
T=[T;t'];
X_CoM=[X_CoM;x'];
Y_CoM=[Y_CoM;y'];
dX_CoM=[dX_CoM;dx'];
ddX_CoM=[ddX_CoM;ddx'];
dY_CoM=[dY_CoM;dy'];
ddY_CoM=[ddY_CoM;ddy'];
sFootX = [sFootX;ones(length(t),1)*px_0];
sFootY = [sFootY;ones(length(t),1)*py_0];       
dFootX = sFootX;
dFootY = sFootY;
%%
for n=1:NumStep
    % Equation 4.50 Boook
    if n==1
        px(n) = px_0+s_x(n);
        py(n) = py_0-(-1)^n*s_y(n);
    else
        px(n) = px(n-1)+s_x(n);
        py(n) = py(n-1)-(-1)^n*s_y(n);
    end  
    % Equation 4.51    
    x_(n) = s_x(n+1)/2;
    y_(n) = (-1)^n*s_y(n+1)/2;
    % Equation 4.52
    dx_(n) = (C+1)/(T_c*S)*x_(n);
    dy_(n) = (C-1)/(T_c*S)*y_(n);
    % Equation 4.57
    x_d(n) = px(n)+x_(n);
    dx_d(n) = dx_(n);        
    y_d(n) = py(n)+y_(n);
    dy_d(n) = dy_(n);
    
    % Equation 4.59
    px_mod(n) = -a*(C-1)/D*(x_d(n)-C*x_init-T_c*S*dx_init)- ...
                b*S/T_c/D*(dx_d(n)-S/T_c*x_init-C*dx_init);      
    py_mod(n) = -a*(C-1)/D*(y_d(n)-C*y_init-T_c*S*dy_init)- ...
                b*S/T_c/D*(dy_d(n)-S/T_c*y_init-C*dy_init);
            
    % Begining Double support phase
    xds_begin = x(end);
    yds_begin = y(end);
    dxds_begin = dx(end);
    dyds_begin = dy(end);
    ddxds_begin = ddx(end);
    ddyds_begin = ddy(end);
   
    % Ending Double support phase
    xds_end = (x_init - px_mod(n))*cosh(Tds/2/T_c) +...
            T_c*dx_init*sinh(Tds/2/T_c) + px_mod(n);
    yds_end = (y_init - py_mod(n))*cosh(Tds/2/T_c) +...
            T_c*dy_init*sinh(Tds/2/T_c) + py_mod(n);
    dxds_end = (x_init - px_mod(n))/T_c*sinh(Tds/2/T_c) +...
            dx_init*cosh(Tds/2/T_c);
    dyds_end = (y_init - py_mod(n))/T_c*sinh(Tds/2/T_c) +...
            dy_init*cosh(Tds/2/T_c);
    ddxds_end = (x_init-px_mod(n))/T_c^2*cosh(Tds/2/T_c)+...
            dx_init/T_c*sinh(Tds/2/T_c);
    ddyds_end = (y_init - py_mod(n))/T_c^2*cosh(Tds/2/T_c)+...
            dy_init/T_c*sinh(Tds/2/T_c);
    
    % In double support phase
    % tds = 0:dtime:Tds
    % For xCoM
    Ax = [0         0           0           0       1; % x(0)          
          0         0           0           1       0; % dx(0)
          4*Tds^3   3*Tds^2     2*Tds       1       0; % dx(Tds)
          0         0           2           0       0; % ddx(0)
          12*Tds^2  6*Tds       2           0       0];% ddx(Tds)  
    Bx = [xds_begin;
          dxds_begin;
          dxds_end;
          ddxds_begin;
          ddxds_end];
    axi = Ax\Bx;

    xds = poly4th(axi,dt:dt:Tds);
    dxds = dpoly4th(axi,dt:dt:Tds);
    ddxds = ddpoly4th(axi,dt:dt:Tds);
    
    % For yCoM
    Ay = [0         0           0           0       1; % y(0)
          0         0           0           1       0; % dy(0)
          4*Tds^3   3*Tds^2     2*Tds       1       0; % dy(Tds)
          0         0           2           0       0; % ddy(0)
          12*Tds^2  6*Tds       2           0       0];% ddy(Tds) 
    By = [yds_begin;
          dyds_begin;
          dyds_end;
          ddyds_begin;
          ddyds_end];
    ayi = Ay\By;
    yds = poly4th(ayi,dt:dt:Tds);
    dyds = dpoly4th(ayi,dt:dt:Tds);
    ddyds = ddpoly4th(ayi,dt:dt:Tds);
    
       
%   Saving inital value for next step
    x_init = xds(end);
    y_init = yds(end);  
    dx_init = dxds_end;
    dy_init = dyds_end;
    
    px_mod(n) = -a*(C-1)/D*(x_d(n)-C*x_init-T_c*S*dx_init)- ...
                b*S/T_c/D*(dx_d(n)-S/T_c*x_init-C*dx_init);      
    py_mod(n) = -a*(C-1)/D*(y_d(n)-C*y_init-T_c*S*dy_init)- ...
                b*S/T_c/D*(dy_d(n)-S/T_c*y_init-C*dy_init);
    
    t = (dt):dt:Tss;
    x =(x_init - px_mod(n))*cosh(t/T_c) + T_c*dx_init*sinh(t/T_c) + px_mod(n);
    dx =(x_init - px_mod(n))/T_c*sinh(t/T_c) + dx_init*cosh(t/T_c);
    ddx = (x_init - px_mod(n))/T_c^2*cosh(t/T_c) + dx_init/T_c*sinh(t/T_c);
    y =(y_init - py_mod(n))*cosh(t/T_c) + T_c*dy_init*sinh(t/T_c) + py_mod(n);
    dy =(y_init - py_mod(n))/T_c*sinh(t/T_c) + dy_init*cosh(t/T_c);  
    ddy = (y_init - py_mod(n))/T_c^2*cosh(t/T_c) + dy_init/T_c*sinh(t/T_c);
    
    % Saving inital value for next step
    x_init = x(end);
    y_init = y(end);
    dx_init = dx(end);
    dy_init = dy(end);
    
    % Store value
    tmptimeds = n*Tss+(n-1)*Tds+tds(2:end)';
    tmptimess = n*(Tss+Tds)+t';
    T=[T;tmptimeds;tmptimess];
    X_CoM=[X_CoM;xds';x'];
    Y_CoM=[Y_CoM;yds';y'];
    dX_CoM=[dX_CoM;dxds';dx'];
    dY_CoM=[dY_CoM;dyds';dy'];
    ddX_CoM=[ddX_CoM;ddxds';ddx'];
    ddY_CoM=[ddY_CoM;ddyds';ddy'];
    temp=[n*Tss+tds(2:end)';n*(Tss+Tds)+t'];
    sFootX = [sFootX;ones(length(temp),1)*px_mod(n)];
    sFootY = [sFootY;ones(length(temp),1)*py_mod(n)];
    dFootX = [dFootX;ones(length(temp),1)*px(n)];
    dFootY = [dFootY;ones(length(temp),1)*py(n)];
end
%% Final walking state
% Begining Double support phase
n = n+1;
xds_begin = x(end);
yds_begin = y(end);
dxds_begin = dx(end);
dyds_begin = dy(end);
ddxds_begin = ddx(end);
ddyds_begin = ddy(end);
% end double support
ddxds_end = 0;
dxds_end = 0;
dyds_end = 0;
ddyds_end = 0;

% In double support phase
% tds = 0:dtime:Tds
% For xCoM
Ax = [0         0           0           0       1; % x(0)          
      0         0           0           1       0; % dx(0)
      4*Tds^3   3*Tds^2     2*Tds       1       0; % dx(Tds)
      0         0           2           0       0; % ddx(0)
      12*Tds^2  6*Tds       2           0       0];% ddx(Tds)  
Bx = [xds_begin;
      dxds_begin;
      dxds_end;
      ddxds_begin;
      ddxds_end];
axi = Ax\Bx;

xds = poly4th(axi,dt:dt:Tds);
dxds = dpoly4th(axi,dt:dt:Tds);
ddxds = ddpoly4th(axi,dt:dt:Tds);
   
% For yCoM
Ay = [0         0           0           0       1; % y(0)
      0         0           0           1       0; % dy(0)
      4*Tds^3   3*Tds^2     2*Tds       1       0; % dy(Tds)
      0         0           2           0       0; % ddy(0)
      12*Tds^2  6*Tds       2           0       0];% ddy(Tds) 
By = [yds_begin;
      dyds_begin;
      dyds_end;
      ddyds_begin;
      ddyds_end];
ayi = Ay\By;
yds = poly4th(ayi,dt:dt:Tds);
dyds = dpoly4th(ayi,dt:dt:Tds);
ddyds = ddpoly4th(ayi,dt:dt:Tds);

px(n) = px(n-1)+s_x(n);
py(n) = py(n-1)-(-1)^n*s_y(n);

tmptimeds = n*Tss+(n-1)*Tds+tds(2:end)';
T=[T;tmptimeds];

% Store value
X_CoM=[X_CoM;xds'];
Y_CoM=[Y_CoM;yds'];
dX_CoM=[dX_CoM;dxds'];
dY_CoM=[dY_CoM;dyds'];
ddX_CoM=[ddX_CoM;ddxds'];
ddY_CoM=[ddY_CoM;ddyds'];

sFootX = [sFootX;ones(length(tmptimeds),1)*X_CoM(end)];
sFootY = [sFootY;ones(length(tmptimeds),1)*(2*Y_CoM(end)-sFootY(end))];
dFootX = [dFootX;ones(length(tmptimeds),1)*px(n)];
dFootY = [dFootY;ones(length(tmptimeds),1)*py(n)];
%% End Final walking state

RFootXY =[0 -0.1];
LFootXY =[0  0.1];
for idx=2:length(px_mod)
    if mod(idx,2)==0
        RFootXY = [RFootXY;px_mod(idx) py_mod(idx)];
    else
        LFootXY = [LFootXY;px_mod(idx) py_mod(idx)];
    end    
end
RFootXY = [RFootXY;sFootX(end) sFootY(end)];
%% Foot Trajectory

RightX = [];
RightY = [];
RightZ = [];
LeftX = [];
LeftY = [];
LeftZ = [];
RightFoot =[];
endtime = 0;
Rtimes = [];
Ltimes = [];
starttime = 0.8;
RightTime_init = (0:dt:starttime-dt)';
RightX_init = zeros(length(RightTime_init),1);
testtime = [];
%% RIGHT FOOT
for idx=2:length(RFootXY)
    [ttmp,xtmp] = Foot_X(RFootXY(idx-1,1),RFootXY(idx,1),starttime,dt);
    [~,ztmp] = Foot_Z(starttime,dt);    
    RightX = [RightX;xtmp];
    RightY = [RightY;linspace(RFootXY(idx-1,2),RFootXY(idx,2),length(ttmp))'];
    RightZ = [RightZ;ztmp];    
    Rtimes = [Rtimes;ttmp];
    
    endtime = Rtimes(end);
    if idx~=length(RFootXY)
        RightX(end)=[];
        RightY(end)=[];
        RightZ(end)=[];
        Rtimes(end)=[];
        tmp = Rtimes(end)+(dt:dt:Tss)';        
        RightX = [RightX;RightX(end)*ones(length(tmp),1)];
        RightY = [RightY;RightY(end)*ones(length(tmp),1)];
        RightZ = [RightZ;RightZ(end)*ones(length(tmp),1)];
        Rtimes = [Rtimes;tmp];
    end    
    starttime = endtime+Tss;    
end
RightX = [RightX_init;RightX];
RightZ_init = 60*ones(length(RightTime_init),1);
RightY_init = RightY(1)*ones(length(RightTime_init),1);
RightZ = [RightZ_init;RightZ];
RightY = [RightY_init;RightY];
Rtimes = [RightTime_init;Rtimes];
%% lEFT FOOT
starttime = 0.8+Tss;
LeftTime_init = (0:dt:starttime-dt)';
LeftX_init = zeros(length(0:dt:starttime-dt),1);

for idx=2:length(LFootXY)
    [ttmp,xtmp] = Foot_X(LFootXY(idx-1,1),LFootXY(idx,1),starttime,dt);
    [~,ztmp] = Foot_Z(starttime,dt);
    LeftX = [LeftX;xtmp];
    LeftY = [LeftY;linspace(LFootXY(idx-1,2),LFootXY(idx,2),length(ttmp))'];
    LeftZ = [LeftZ;ztmp];
    Ltimes = [Ltimes;ttmp]; 
    
    endtime = Ltimes(end);
    if idx~=length(LFootXY)
        LeftX(end)=[];
        LeftY(end)=[];
        LeftZ(end)=[];
        Ltimes(end)=[]; 
%         testtime = [testtime;Ltimes(end)+(dtime:dtime:Tss)'];
        tmp = Ltimes(end)+(dt:dt:Tss)';        
        LeftX = [LeftX;LeftX(end)*ones(length(tmp),1)];
        LeftY = [LeftY;LeftY(end)*ones(length(tmp),1)];
        LeftZ = [LeftZ;LeftZ(end)*ones(length(tmp),1)];
        Ltimes = [Ltimes;tmp];
    end    
    starttime = endtime+Tss;
end
LeftX = [LeftX_init;LeftX];
LeftZ_init = 80*ones(length(LeftTime_init),1);
LeftY_init = LeftY(1)*ones(length(LeftTime_init),1);
LeftZ = [LeftZ_init;LeftZ];
LeftY = [LeftY_init;LeftY];
Ltimes = [LeftTime_init;Ltimes];
% When Biped stop, left leg stance
LeftTime_end = (Ltimes(end)-dt:dt:Rtimes(end))';%
LeftTime_end = LeftTime_end(3:end);
LeftX_end = LeftX(end)*ones(length(LeftTime_end),1);
LeftZ_end = 80*ones(length(LeftTime_end),1);
LeftY_end = LeftY(end)*ones(length(LeftTime_end),1);
LeftZ = [LeftZ;LeftZ_end];
LeftY = [LeftY;LeftY_end];
LeftX = [LeftX;LeftX_end];
Ltimes = [Ltimes;LeftTime_end];

zmpx = X_CoM(1) - (ZCoM - 0)*ddX_CoM(1)/(0+9810);
zmpy = Y_CoM(1) - (ZCoM - 0)*ddY_CoM(1)/(0+9810);
for idx = 2:length(Rtimes)
    zmpx = [zmpx; X_CoM(idx) - (ZCoM - 0)*ddX_CoM(idx)/(0+9810)];
    zmpy = [zmpy ;Y_CoM(idx) - (ZCoM - 0)*ddY_CoM(idx)/(0+9810)];
end

times = Rtimes;
LeftFoot = [LeftX LeftY LeftZ];
RightFoot = [RightX RightY RightZ];
BipedCoM = [X_CoM Y_CoM ones(length(times),1)*ZCoM];
ZmpDesired = [zmpx zmpy];
% Plot
figure
subplot(3,1,1);
plot(times,BipedCoM(:,1),'-r');
grid on
subplot(3,1,2);
plot(times,BipedCoM(:,2),'-r');
grid on
subplot(3,1,3);
plot(times,BipedCoM(:,3),'-r');
grid on
figure
subplot(2,1,1)
hold on
plot(times,sFootX,'-r');
plot(times,zmpx,'-b');
grid on
subplot(2,1,2)
hold on
plot(times,sFootY,'-r');
plot(times,zmpy,'-b');
grid on

