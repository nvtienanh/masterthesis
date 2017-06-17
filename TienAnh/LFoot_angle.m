function Th_a = RFoot_angle(t)
%Foot_angle.m
%Function generation foot_angle
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
    global Tc Td qb qf qs qe Dtime
    k = 0;
%     Dtime = 0.001;
    th_time=[k*Tc;k*Tc+Td;(k+1)*Tc;(k+1)*Tc+Td];
    th_val=[qs;qb;qf;qe];
    % theta foot;
    % AX=y
    t1 = th_time(1);
    t2 = th_time(2);
    t3 = th_time(3);
    t4 = th_time(4);
    f1 = th_val(1);
    f2 = th_val(2);
    f3 = th_val(3);
    f4 = th_val(4);
    A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0;...
       t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0;...
       0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1;...
       0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
       3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0;...
       3*t2^2,2*t2,1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,3*t2^2,2*t2,1,0,0,0,0,0;...
       0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t3^2,2*t3,1,0;...
       0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0];
%     A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0;...
%        t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
%        0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0;...
%        0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
%        0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1;...
%        0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
%        3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0;...
%        3*t2^2,2*t2,1,0,0,0,0,0,0,0,0,0;...
%        0,0,0,0,3*t2^2,2*t2,1,0,0,0,0,0;...
%        0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0;...
%        0,0,0,0,0,0,0,0,3*t3^2,2*t3,1,0;...
%        0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0];
    Y = [f1;f2;f2;f3;f3;f4;0;0;0;0;0;0];
    X = A\Y;
    th_a = zeros(3,4);
    k=0;
    for i=1:3
        th_a(i,:)=[X(k+1),X(k+2),X(k+3),X(k+4)];
        k = k+4;    
    end
    times=[];
    angle=[];
    for i=1:length(th_time)-1
        y_1 = @(x) th_a(i,1)*x.^3+th_a(i,2)*x.^2+th_a(i,3)*x+th_a(i,4);
        x_1 = th_time(i):Dtime:th_time(i+1);
         % Remove repeat value
        if i<length(th_time)-1
            x_1(end)=[];
        end
        times=[times;x_1'];
        temp = y_1(x_1);
        angle=[angle;temp'];        
    end
    temp = times(end):Dtime:2*Tc;
    temp(1)=[];
    temp(end)=[];
    times=[times;temp'];
    angle=[angle;zeros(length(temp),1)];
%     th_a=[times,angle];
    
    numCycle = round(t/2/Tc+0.5);
    oneCycle = times;
    oneAngle = angle;
    for i=2:numCycle
        oneCycle = 2*Tc+oneCycle;
        temp = oneCycle;
        temp(1)=[];
        times=[times;temp];
        temp = oneAngle;
        temp(1)=[];
        angle = [angle;temp];
    end
    timewalk = 0:Dtime:t;
    len = length(timewalk);
    times = times(1:len+1);
    angle = angle(1:len+1);
    Th_a = [times angle];
end

