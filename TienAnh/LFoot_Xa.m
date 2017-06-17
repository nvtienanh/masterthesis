function X_a = LFoot_Xa(t)
% Foot_Xa.m
% Function generation Foot_Xa
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
    global Tc Td To qb qf Lao lan Ds lab laf Dtime
    k = 0;
%     Dtime = 0.001;
    x_val=[k*Ds;...
            k*Ds+laf*(1-cos(qb))+lan*sin(qb);...
            k*Ds+Lao;...
            (k+1)*Ds-lab*(1-cos(qf))-lan*sin(qf);...
            (k+1)*Ds];
    x_time=[k*Tc;...
            k*Tc+Td;...
            k*Tc+To;...
            (k+1)*Tc;...
            (k+1)*Tc+Td];
    % X_a foot;
    % AX=y
    t1 = x_time(1);
    t2 = x_time(2);
    t3 = x_time(3);
    t4 = x_time(4);
    t5 = x_time(5);
    f1 = x_val(1);
    f2 = x_val(2);
    f3 = x_val(3);
    f4 = x_val(4);
    f5 = x_val(5);
    A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
       0,0,0,0,0,0,0,0,0,0,0,0,t5^3,t5^2,t5,1;...
       3*t2^2,2*t2,1,0,-3*t2^2,-2*t2,-1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,3*t3^2,2*t3,1,0,-3*t3^2,-2*t3,-1,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0,-3*t4^2,-2*t4,-1,0;...
       3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0,0,0,0,0;...
%        0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0,0,0,0,0;...
%        0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0,0,0,0,0;...
       6*t2,2,0,0,-6*t2,-2,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,6*t3,2,0,0,-6*t3,-2,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,6*t4,2,0,0,-6*t4,-2,0,0;...      
%        6*t1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,3*t5^2,2*t5,1,0];
%        0,0,0,0,0,0,0,0,0,0,0,0,6*t5,2,0,0];
    Y = [f1;f2;f2;f3;f3;f4;f4;f5;0;0;0;0;0;0;0;0];
    X = A\Y;
    x_a = zeros(length(x_time)-1,4);
    k=0;
    for i=1:length(x_time)-1;
        x_a(i,:)=[X(k+1),X(k+2),X(k+3),X(k+4)];
        k = k+4;    
    end
    times=[];
    Xa=[];
    for i=1:length(x_time)-1
        y_1 = @(x) x_a(i,1)*x.^3+x_a(i,2)*x.^2+x_a(i,3)*x+x_a(i,4);
        x_1 = x_time(i):Dtime:x_time(i+1);
        % Remove repeat value
        if i<length(x_time)-1
            x_1(end)=[];
        end
        times=[times;x_1'];
        temp = y_1(x_1);
        Xa=[Xa;temp'];
    end
    temp = times(end):Dtime:2*Tc;
    temp(1)=[];
    temp(end)=[];
    times=[times;temp'];
    Xa=[Xa;ones(length(temp),1)*Ds];

    numCycle = round(t/2/Tc+0.5);
    oneCycle = times;
    idx = find(times==Tc);    
    tmp = [Xa(idx:end)-0.5*Ds;Xa(1:idx-1)+0.5*Ds];
    Xa = tmp;
    oneX = Xa;
    clear tmp
    for i=2:numCycle
        oneCycle = 2*Tc+oneCycle;
        temp = oneCycle;
        temp(1)=[];
        times=[times;temp];
        oneX = oneX+Ds;
        temp = oneX;
        temp(1)=[];
        Xa = [Xa;temp];
    end
    timewalk = 0:Dtime:t;
    len = length(timewalk);
    times = times(1:len+1);
    Xa = Xa(1:len+1);
    X_a = [times Xa];
end

