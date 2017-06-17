function [CoM,dCoM,ZMP,RightAnkle,LeftAnkle,Tpattern,ZMP_x,ZMP_y]= CalPattern(ZCoM,Tcy,Tsamp,doplot)
%% CalPattern.m
% Function Pattern generator CoM and Foot Placement without Foot trajectory.
% In this code add double support phase
% IN THIS CODE, THE WALKING DESIGN ASSUME:
% - THE FIRST SWING LEG IS RIGHT LEG
% - THE LAST LANDING LEG IS LEFT LEG
% - I ADD SOME STUPID CODE IN DEVPHASE WALKING AND DECAY PHASE WALKING
% INPUT:
% 1) ZCoM: Height of CoM
% 2) Tcy: Time of one walking cycle
% 3) Tsamp: Sampling time
% 4) doplot: if true
%               plotting some useful graph
%            else do not plot
% OUTPUT:
% 1) CoM: Center of Mass trajectory
% 2) dCoM: Center of Mass veclocity
% 3) RightAnkle: Right Ankle trajectory
% 4) LeftAnkle : Left Ankle trajectory
% 5) Tpattern: Timeseries of Pattern generation
% Nguyen Van Tien Anh
% 26/02/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GLOBAL VARIABLE
global Tsup Tss Tds Tsampling Lan
Tsup = Tcy;
Tss = 0.6*Tsup;
Tds = 0.4*Tsup;
Tsampling = Tsamp;
Lan = 0.065;
%--------------------------------------------------------------------------
g = 9.81;
T_c = sqrt(ZCoM/g);
walkstep = 0.1;% m
HipWidth = 0.114;% m
%% Walk Phase based on page 34
% J. Rose and J. G. Gamble, Eds., Human walking, 3rd ed. Philadelphia: 
% Lippincott Williams & Wilkins, 2006J. Rose and J. G. Gamble, Eds., Human 
% walking, 3rd ed. Philadelphia: Lippincott Williams & Wilkins, 2006
% Development Phase !! DO NOT MODIFY
DevPhase = [0      walkstep/2;
            HipWidth HipWidth];
% Rhythmic phase !! CAN MODIFY BUT length(RhyPhase) is a odd number
RhyPhase = [walkstep walkstep walkstep;
            HipWidth HipWidth HipWidth];
% Decay phase !! DO NOT MODIFY
DecPhase = [walkstep/2 0 0;
            HipWidth HipWidth/2 0];
WalkDesign = [DevPhase RhyPhase DecPhase];        
s_x = WalkDesign(1,:);
s_y = WalkDesign(2,:);
NumStep = length(WalkDesign);

%% Step 1
% CoM inititak when Biped In Double support phase
% Initial CoM
CoM_init = [0 0]; % CoM initial position when start walking
dCoM_init = [0 0];
% Stance Foot placement
LeftFoot_init = [0 0.114/2]; % Left Foot Initial Position
RightFoot_init = [0 -0.114/2]; % Right Foot Initial Position
StanceFoot_init = RightFoot_init;   % Stance Foot placement Desired initial Right Foot       
StanceFootMod_init = StanceFoot_init; % Stance Foot placement Modify initital
% Calculate time
Tcycle = Tsampling:Tsampling:Tsup;Tcycle = Tcycle'; % Time of one walking cycle
NumOneCycle = length(Tcycle); % Number of Sampling data in one Cycle Single Support
% Inital variable for calculate
CoM = zeros(NumOneCycle*NumStep+1,2);
dCoM = zeros(NumOneCycle*NumStep+1,2);
Timewalk = zeros(NumOneCycle*NumStep+1,1); % Walking Time - Initial variable
StanceFoot = zeros(NumStep,2); % Stance Foot placement Desired
StanceFootMod = zeros(NumStep,2); % Stance Foot placement
RightFoot = zeros(NumStep,2);
LeftFoot  = zeros(NumStep,2);
% Initial Condition
CoM(1,:) = CoM_init; % CoM position at t = 0;
dCoM(1,:) = dCoM_init; % CoM position velocity at t = 0;
a = 10;
b = 1;    
C = cosh(Tsup/T_c);
S = sinh(Tsup/T_c);
D = a*(C-1)^2+b*(S/T_c)^2;
           
for n=0:NumStep-1
    if n==0        
        StanceFoot(n+1,1) = StanceFoot_init(1);
        StanceFoot(n+1,2) = StanceFoot_init(2);
        % Equation 4.51
        xdash = s_x(n+1)/2;
        ydash = (-1)^n*s_y(n+1)/2;
        % Equation 4.52
        dxdash = (C+1)/(T_c*S)*xdash;
        dydash = (C-1)/(T_c*S)*ydash;
        % Equation 4.57
        xd = StanceFoot(n+1,1) + xdash;
        yd = StanceFoot(n+1,2) + ydash;
        dxd = dxdash;
        dyd = dydash;    
        % Equation 4.59
        StanceFootMod(n+1,1) = -a*(C-1)/D*(xd-C*CoM_init(1)-T_c*S*dCoM_init(1))- ...
            b*S/T_c/D*(dxd-S/T_c*CoM_init(1)-C*dCoM_init(1));      
        StanceFootMod(n+1,2) = -a*(C-1)/D*(yd-C*CoM_init(2)-T_c*S*dCoM_init(2))- ...
            b*S/T_c/D*(dyd-S/T_c*CoM_init(2)-C*dCoM_init(2));   
        % Calculate Right Foot and Left Foot Placement
        RightFoot(n+1,:) = RightFoot_init;
        LeftFoot(n+1,:) = LeftFoot_init;
    else
        % Equation 4.50
        StanceFoot(n+1,1) = StanceFoot_init(1) + s_x(n);
        StanceFoot(n+1,2) = StanceFoot_init(2) - (-1)^n*s_y(n);
        % Equation 4.51
        xdash = s_x(n+1)/2;
        ydash = (-1)^n*s_y(n+1)/2;
        % Equation 4.52
        dxdash = (C+1)/(T_c*S)*xdash;
        dydash = (C-1)/(T_c*S)*ydash;
        % Equation 4.57
        xd = StanceFoot(n+1,1) + xdash;
        yd = StanceFoot(n+1,2) + ydash;
        dxd = dxdash;
        dyd = dydash;    
        % Equation 4.59
        StanceFootMod(n+1,1) = -a*(C-1)/D*(xd-C*CoM_init(1)-T_c*S*dCoM_init(1))- ...
            b*S/T_c/D*(dxd-S/T_c*CoM_init(1)-C*dCoM_init(1));      
        StanceFootMod(n+1,2) = -a*(C-1)/D*(yd-C*CoM_init(2)-T_c*S*dCoM_init(2))- ...
            b*S/T_c/D*(dyd-S/T_c*CoM_init(2)-C*dCoM_init(2));   
        % Calculate Right Foot and Left Foot Placement
        if mod(n,2)
            RightFoot(n+1,:) = RightFoot(n,:);
            LeftFoot(n+1,:) = StanceFootMod(n+1,:);
        else
            RightFoot(n+1,:) = StanceFootMod(n+1,:);
            LeftFoot(n+1,:) = LeftFoot(n,:);
        end
        
    end    
    % Equation 4.54 & 4.55
    x = (CoM_init(1) - StanceFootMod(n+1,1))*cosh(Tcycle/T_c) + ...
        T_c*dCoM_init(1)*sinh(Tcycle/T_c) + StanceFootMod(n+1,1);
    dx = (CoM_init(1) - StanceFootMod(n+1,1))*sinh(Tcycle/T_c)/T_c + ...
        dCoM_init(1)*cosh(Tcycle/T_c);

    y = (CoM_init(2) - StanceFootMod(n+1,2))*cosh(Tcycle/T_c) + ...
        T_c*dCoM_init(2)*sinh(Tcycle/T_c) + StanceFootMod(n+1,2);
    dy = (CoM_init(2) - StanceFootMod(n+1,2))*sinh(Tcycle/T_c)/T_c + ...
        dCoM_init(2)*cosh(Tcycle/T_c);
    % Save data
    Timewalk(n*NumOneCycle+2:(n+1)*NumOneCycle+1) = n*Tsup+Tcycle;
    CoM(n*NumOneCycle+2:(n+1)*NumOneCycle+1,:) = [x y];
    dCoM(n*NumOneCycle+2:(n+1)*NumOneCycle+1,:) = [dx dy];
    CoM_init = [x(end) y(end)];
    dCoM_init = [dx(end) dy(end)];
    StanceFoot_init = StanceFoot(n+1,:);
    
end
%% Add final step
LeftFoot(end,:) = [RightFoot(end,1) -RightFoot(end,2)];
% Export ZMP desired
ZMP = zeros(length(Timewalk),2);
ZMP(1:NumOneCycle+1,:) = zeros(NumOneCycle+1,2);
for idx=2:NumStep    
    idxBegin = (idx-1)*NumOneCycle+2;
    idxEnd = idx*NumOneCycle+1;
    ZMP(idxBegin:idxEnd,:) = repmat(StanceFoot(idx,:),NumOneCycle,1);
end

%% Adding Double support phase to Walking cycle
% Calculate time
% Follow Equation 4.63 and interpolate CoM at Double Phase at t= [t1 t2]
% while t1 = 0 and t2 = Tds and tm = (t1+t2)/2;
% x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
% x(t1) = CoM at t = Tss
% x(t2) = CoM at t = Tss
% dx(t1) = dCoM at t = Tss
% dx(t2) = dCoM at t = Tss
% ddx(tm) = 0
% 
% AX = B
% 
TimeDS = 0:Tsampling:Tds; TimeDS = TimeDS';
TimeSS = Tsampling:Tsampling:Tss; TimeSS = TimeSS';
NumOneDS = length(TimeDS)-1;
NumOneSS = length(TimeSS);
% NumOneCycle = NumOneDS+NumOneCycle;
Tm = Tds/2;
A = [1  0       0       0           0;
     1  Tds     Tds^2   Tds^3       Tds^4;
     0  1       0       0           0;
     0  1       2*Tds   3*Tds^2     4*Tds^3;
     0  0       2       6*Tm        12*Tm^2];
%% Anonymous Function 
poly4th = @(coeff,x) coeff(5)*x.^4+coeff(4)*x.^3+coeff(3)*x.^2+...
                coeff(2)*x+coeff(1);
dpoly4th = @(coeff,x) 4*coeff(5)*x.^3+3*coeff(4)*x.^2+2*coeff(3)*x+coeff(2);  

CoMDS = CoM;
dCoMDS = dCoM;
for n=1:NumStep-1
    idx = n*NumOneCycle+1;
    idxDSBegin = idx - NumOneDS/2; % Index Begin Double Support phase
    idxDSEnd = idx + NumOneDS/2; % Index End Double support phase
    
    Bx = [CoM(idxDSBegin,1);
          CoM(idxDSEnd,1);
          dCoM(idxDSBegin,1)
          dCoM(idxDSEnd,1);
          0];
    By = [CoM(idxDSBegin,2);
          CoM(idxDSEnd,2);
          dCoM(idxDSBegin,2)
          dCoM(idxDSEnd,2);
          0];  
    ax = A\Bx;    %axi = Ax\Bx;
    ay = A\By;    %axi = Ax\Bx;
    tmpCoMx = poly4th(ax,TimeDS);
    tmpdCoMx = dpoly4th(ax,TimeDS);    
    tmpCoMy = poly4th(ay,TimeDS);
    tmpdCoMy = dpoly4th(ay,TimeDS);  
    % CoM Position
    CoMDS(idxDSBegin:idxDSEnd,:) = [tmpCoMx tmpCoMy];    
    % CoM Velocity       
    dCoMDS(idxDSBegin:idxDSEnd,:) = [tmpdCoMx tmpdCoMy];    
end
%%
%% STUPID CODE ------------------------------------------------------------
LeftFoot(2:3,:) = repmat(LeftFoot(1,:),2,1);
%% Calculate Footplacement
LeftFootPos = zeros(length(Timewalk),2);
RightFootPos = zeros(length(Timewalk),2);
for idx=1:NumStep
    idxbegin = (idx-1)*NumOneCycle+1;
    idxend   = idxbegin+NumOneCycle;    
    LeftFootPos(idxbegin:idxend,:) = repmat(LeftFoot(idx,:),NumOneCycle+1,1);
    RightFootPos(idxbegin:idxend,:) = repmat(RightFoot(idx,:),NumOneCycle+1,1);
end
%--------------------------------------------------------------------------
%% Foot Trajectory
% From Walking Pattern Design, the fist swing leg is right leg and left leg
% is stance leg.
% NumStep = 1: Initital Step, two leg in Double Support phase
% when t = 0:Tss. Double support phase. CoM shifts to initial position for
% begining walking
% at t = Tss: Tss+Tds Right Foot is starting move
%
%%
RightAnkle = zeros(length(Timewalk),3);
LeftAnkle = zeros(length(Timewalk),3);
for n=0:NumStep-2
    if n==0        
        % Initial Walking step
        idxBegin = n*NumOneCycle+1; % Index Begin Double Support phase
        idxEnd = idxBegin + NumOneSS+NumOneDS/2-1; % Index End Double support phase
        OnesTmp = ones(NumOneSS+NumOneDS/2,1);        
        RightAnkle(idxBegin:idxEnd,1) = OnesTmp*RightFoot(n+1,1);
        RightAnkle(idxBegin:idxEnd,2) = OnesTmp*RightFoot(n+1,2);
        RightAnkle(idxBegin:idxEnd,3) = OnesTmp*Lan; % Lan = 50         
        OnesTmp = ones(NumOneCycle+NumOneDS/2,1); 
        LeftAnkle(idxBegin:idxEnd+NumOneDS,1) = OnesTmp*LeftFoot(n+1,1);
        LeftAnkle(idxBegin:idxEnd+NumOneDS,2) = OnesTmp*LeftFoot(n+1,2);
        LeftAnkle(idxBegin:idxEnd+NumOneDS,3) = OnesTmp*Lan; % Lan = 50        
    elseif mod(n,2) % Right Leg Swing - 1
                    % Left Leg Stance - 0
        idx = n*NumOneCycle+1;
        idxBegin = idx-NumOneDS/2; % Index Begin Ankle Cycle        
        idx = (n+1)*NumOneCycle+1;
        idxEnd = idx + NumOneDS/2; % Index End Ankle Cycle        
        OnesTmp = ones(NumOneSS+1,1);
        [rtimes,xfoot] = CalFootX(RightFoot(n+1,1),RightFoot(n+2,1),Timewalk(idxBegin));
        [~,zfoot] = CalFootZ(Timewalk(idxBegin));       
        RightAnkle(idxBegin:idxEnd,1) = xfoot;
        RightAnkle(idxBegin:idxEnd,2) = repmat(RightFoot(n+1,2),length(xfoot),1);        
        RightAnkle(idxBegin:idxEnd,3) = zfoot;         
        LeftAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,1) = OnesTmp*LeftFoot(n+1,1);
        LeftAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,2) = OnesTmp*LeftFoot(n+1,2);
        LeftAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,3) = OnesTmp*Lan; % Lan = 50        
    else % Right Leg Stance - 0
         % Left Leg Swing   - 1    
        idx = n*NumOneCycle+1;
        idxBegin = idx-NumOneDS/2; % Index Begin Ankle Cycle
        idx = (n+1)*NumOneCycle+1;
        idxEnd = idx + NumOneDS/2; % Index End Ankle Cycle
        OnesTmp = ones(NumOneSS+1,1);
        [ltimes,xfoot] = CalFootX(LeftFoot(n+1,1),LeftFoot(n+2,1),Timewalk(idxBegin));        
        [~,zfoot] = CalFootZ(Timewalk(idxBegin));
        LeftAnkle(idxBegin:idxEnd,1) = xfoot;
        LeftAnkle(idxBegin:idxEnd,2) = repmat(LeftFoot(n+1,2),length(xfoot),1);
        LeftAnkle(idxBegin:idxEnd,3) = zfoot;        
        RightAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,1) = OnesTmp*RightFoot(n+1,1);
        RightAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,2) = OnesTmp*RightFoot(n+1,2);
        RightAnkle(idxBegin+NumOneDS:idxEnd-NumOneDS,3) = OnesTmp*Lan; % Lan = 50        
    end
end
%% Stupid code ------------------------------------------------------------
idx = rtimes(end)/Tsampling;
idx=round(idx);
tmpmat = repmat(RightAnkle(idx-1,:),length(Timewalk)-idx+1,1);
RightAnkle(idx:end,:)=tmpmat;
idx = ltimes(end)/Tsampling;
idx=round(idx);
tmpmat = repmat(LeftAnkle(idx-1,:),length(Timewalk)-idx+1,1);
LeftAnkle(idx:end,:)=tmpmat;
%--------------------------------------------------------------------------
%% CALCULATE ZMP STABLE MARGIN
ZMP_x = zeros(length(Timewalk),2);
ZMP_y = zeros(length(Timewalk),2);
for idx=1:length(Timewalk)
    if RightAnkle(idx,3)<=Lan && LeftAnkle(idx,3)<=Lan % Double support phase
        ZMP_x(idx,1) = min([LeftAnkle(idx,1)-0.1;LeftAnkle(idx,1)+0.1;...
            RightAnkle(idx,1)-0.1;RightAnkle(idx,1)+0.1]);
        ZMP_x(idx,2) = max([LeftAnkle(idx,1)-0.1;LeftAnkle(idx,1)+0.1;...
            RightAnkle(idx,1)-0.1;RightAnkle(idx,1)+0.1]);
        ZMP_y(idx,1) = min([LeftAnkle(idx,2)-0.05;LeftAnkle(idx,2)+0.05;...
            RightAnkle(idx,2)-0.05;RightAnkle(idx,2)+0.05]);
        ZMP_y(idx,2) = max([LeftAnkle(idx,2)-0.05;LeftAnkle(idx,2)+0.05;...
            RightAnkle(idx,2)-0.05;RightAnkle(idx,2)+0.05]);
    elseif RightAnkle(idx,3)>Lan % Left Leg is stance
        ZMP_x(idx,1) = LeftAnkle(idx,1)-0.1;                                  
        ZMP_x(idx,2) = LeftAnkle(idx,1)+0.1;
        ZMP_y(idx,1) = LeftAnkle(idx,2)-0.05;                                  
        ZMP_y(idx,2) = LeftAnkle(idx,2)+0.05;
    elseif LeftAnkle(idx,3)>Lan % Right Leg stance
        ZMP_x(idx,1) = RightAnkle(idx,1)-0.1;                                  
        ZMP_x(idx,2) = RightAnkle(idx,1)+0.1;
        ZMP_y(idx,1) = RightAnkle(idx,2)-0.05;                                  
        ZMP_y(idx,2) = RightAnkle(idx,2)+0.05;
    else
        Display('Error---------')
    end 
end

%--------------------------------------------------------------------------
%% PLOT GRAPH
if doplot
    figure % Figure 1: CoM and Foot placement
    title('Figure 1: CoM and Foot placement')
    axis equal
    hold on
    for idx = 1:NumStep
        idxB = (idx-1)*Tsup/Tsampling+1;
        idxE = idxB+Tsup/Tsampling;
        if mod(idx,2)
            plot(CoMDS(idxB:idxE,1), CoMDS(idxB:idxE,2),'-b');
        else
            plot(CoMDS(idxB:idxE,1), CoMDS(idxB:idxE,2),'-r');
        end
    end
    Draw2DFootPlacement([StanceFootMod_init(1) StanceFootMod_init(2)],0.2,0.1,'--k');
    for idx=1:NumStep
        Draw2DFootPlacement([RightFoot(idx,1) RightFoot(idx,2)],0.2,0.1,'-b');
        Draw2DFootPlacement([LeftFoot(idx,1) LeftFoot(idx,2)],0.2,0.1,'-r');
    end
%     for idx=1:length(Timewalk)
%         Draw2DFootPlacement([RightFootPos(idx,1) RightFootPos(idx,2)],0.2,0.1,'-b');
%         Draw2DFootPlacement([LeftFootPos(idx,1) LeftFootPos(idx,2)],0.2,0.1,'-r');
%     end
    grid on

    figure % Figure 2: CoM trajectory
    subplot(2,1,1)
    title('CoM_x')
    hold on
    plot(Timewalk,CoMDS(:,1),'-b');
    grid on
    subplot(2,1,2)
    title('CoM_y')
    hold on
    plot(Timewalk,CoMDS(:,2),'-b');
    grid on
    figure % Figure 3: CoM veclocity    
    subplot(2,1,1)
    hold on
    title('dCoM_x')
    plot(Timewalk,dCoMDS(:,1),'-b');
    grid on
    subplot(2,1,2)
    title('dCoM_y')
    hold on
    plot(Timewalk,dCoMDS(:,2),'-b');
    grid on

    figure % Figure 4: Foot trajectory    
    subplot(3,1,1)
    title('Ankle in x direction')
    hold on
    plot(Timewalk,RightAnkle(:,1),'-b');
    plot(Timewalk,LeftAnkle(:,1),'-r');
    grid on
    subplot(3,1,2)
    title('Ankle in y direction')
    hold on
    plot(Timewalk,RightAnkle(:,2),'-b');
    plot(Timewalk,LeftAnkle(:,2),'-r');
    grid on
    subplot(3,1,3)
    title('Ankle in z direction')
    hold on
    plot(Timewalk,RightAnkle(:,3),'-b');
    plot(Timewalk,LeftAnkle(:,3),'-r');
    grid on
    
    figure % Figure 5: ZMP Desired    
    subplot(2,1,1)   
    title('ZMP_x')
    plot(Timewalk,ZMP(:,1),'-r');
    grid on
    subplot(2,1,2) 
    title('ZMP_y')
    plot(Timewalk,ZMP(:,2),'-r');
    grid on
    
    figure % Figure 6: ZMP Stable Region    
    subplot(2,1,1)  
    title('ZMP Stable Region in x direction')
    hold on
    plot(Timewalk,ZMP_x(:,1),'--r');
    plot(Timewalk,ZMP_x(:,2),'--b');
    plot(Timewalk,ZMP(:,1),'-r');
    grid on
    subplot(2,1,2) 
    title('ZMP Stable Region in y direction')
    hold on  
    plot(Timewalk,ZMP_y(:,1),'--r');
    plot(Timewalk,ZMP_y(:,2),'--b');
    plot(Timewalk,ZMP(:,2),'-r');
    grid on
end
% Output variable
CoM = [CoMDS repmat(ZCoM,length(CoMDS),1)];
dCoM = [dCoMDS zeros(length(dCoMDS),1)];
Tpattern = Timewalk;
% Clear global variable
clear global Tsup Tss Tds Tsampling Lan
end
function [times,xfoot] = CalFootX(xstart,xend,tstart)
%% Function generation Foot_X
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
global Tsup Tds Tsampling
poly4th = @(coeff,x) coeff(1)*x.^4+coeff(2)*x.^3+coeff(3)*x.^2+...
        coeff(4)*x+coeff(5);    
    
Ds = xend - xstart;
Lao = Ds*0.4;
Tao = 0.5*Tsup;
x_val=[0;...   
   Lao;...   
   Ds];
x_time=[0;Tao;Tsup+Tds];

% X_a foot;
% AX=y
t1 = x_time(1);
t2 = x_time(2);
t3 = x_time(3);
f1 = x_val(1);
f2 = x_val(2);
f3 = x_val(3);
A=[t1^4 t1^3 t1^2 t1 1;% f1
   t2^4 t2^3 t2^2 t2 1;% f2
   t3^4 t3^3 t3^2 t3 1;% f3   
   4*t1^3 3*t1^2 2*t1 1 0;% dx(strart) = 0
   4*t3^3 3*t3^2 2*t3 1 0];% dx(end) = 0
Y = [f1;f2;f3;0;0];
X = A\Y;
times=(t1:Tsampling:t3)';
xfoot = xstart+poly4th(X,times);
times = tstart+times;
end
function [times,zfoot] = CalFootZ(tstart)
% Function generation Foot_Z
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
% poly6th = @(coeff,x) coeff(1)*x.^6+coeff(2)*x.^5+coeff(3)*x.^4+...
%         coeff(4)*x.^3+coeff(5)*x.^2+coeff(6)*x+coeff(7);
global Tsup Tds Tsampling Lan
poly4th = @(coeff,x) coeff(1)*x.^4+coeff(2)*x.^3+coeff(3)*x.^2+...
        coeff(4)*x+coeff(5);    
    
Tao = 0.5*Tsup;
% level ground
Hao = 0.05;
z_val=[Lan;...
Hao;...
Lan];
z_time=[0+Tds;Tao;Tsup];
% AX=Y
t1 = z_time(1);
t2 = z_time(2);
t3 = z_time(3);   
f1 = z_val(1);
f2 = z_val(2);
f3 = z_val(3);

A=[t1^4 t1^3 t1^2 t1 1;% f1
   t2^4 t2^3 t2^2 t2 1;% f2
   t3^4 t3^3 t3^2 t3 1;% f3   
   4*t1^3 3*t1^2 2*t1 1 0;% dx(strart) = 0
   4*t3^3 3*t3^2 2*t3 1 0];% dx(end) = 0
Y = [f1;f2;f3;0;0];
X = A\Y;
times=(t1:Tsampling:t3)';
zfoot = poly4th(X,times);
tmp1 = 0:Tsampling:t1-Tsampling;% Because foot do not rotate
tmp2 = t3+Tsampling:Tsampling:Tsup+Tds;
zfoot = [Lan*ones(length(tmp1),1);zfoot;Lan*ones(length(tmp2),1)];
times = [tmp1';times;tmp2'];
times = tstart+times;
% times
end


