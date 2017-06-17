function Z_a = RFoot_Za(t)
% Foot_Za.m
% Function generation foot_Za
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
    global Tc Td To qb qf Hao lan lab laf Dtime
    k = 0;
%     Dtime = 0.001;
    z_val=[lan;...
            lan*cos(qb)+laf*cos(qb);...
            Hao;...
            lan*cos(qf)+lab*sin(qf);...
            lan];
    z_time=[k*Tc;...
            k*Tc+Td;...
            k*Tc+To;...
            (k+1)*Tc;...
            (k+1)*Tc+Td];
    % AX=Y
    t1 = z_time(1);
    t2 = z_time(2);
    t3 = z_time(3);
    t4 = z_time(4);
    t5 = z_time(5);
    f1 = z_val(1);
    f2 = z_val(2);
    f3 = z_val(3);
    f4 = z_val(4);
    f5 = z_val(5);
    A=[t1^3,t1^2,t1,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,t2^3,t2^2,t2,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,t3^3,t3^2,t3,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,t4^3,t4^2,t4,1;...
       0,0,0,0,0,0,0,0,0,0,0,0,t5^3,t5^2,t5,1;...
       3*t2^2,2*t2,1,0,-3*t2^2,-2*t2,-1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t3^2,2*t3,1,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,3*t4^2,2*t4,1,0,-3*t4^2,-2*t4,-1,0;...
       6*t2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,6*t4,2,0,0;...
       3*t1^2,2*t1,1,0,0,0,0,0,0,0,0,0,0,0,0,0;...
       0,0,0,0,0,0,0,0,0,0,0,0,3*t5^2,2*t5,1,0];
    Y = [f1;f2;f2;f3;f3;f4;f4;f5;0;0;0;0;0;0;0;0];
    X = A\Y;
    z_a = zeros(4,4);
    k=0;
    for i=1:4
        z_a(i,:)=[X(k+1),X(k+2),X(k+3),X(k+4)];
        k = k+4;    
    end
%     figure(1)
%     hold on
%     % axis([0 th_time(end) -0.5 0.5]);
%     for i=1:length(z_time)-1
%         y_1 = @(x) z_a(i,1)*x.^3+z_a(i,2)*x.^2+z_a(i,3)*x+z_a(i,4);
%         x_1 = z_time(i):0.001:z_time(i+1);
%         plot(x_1,y_1(x_1),'color','r','LineWidth',2);
%     end
    times=[];
    Z=[];
    for i=1:length(z_time)-1
        y_1 = @(x) z_a(i,1)*x.^3+z_a(i,2)*x.^2+z_a(i,3)*x+z_a(i,4);
        x_1 = z_time(i):Dtime:z_time(i+1);
        % Remove repeat value
        if i<length(z_time)-1
            x_1(end)=[];
        end
        times=[times;x_1'];
        temp = y_1(x_1);
        Z=[Z;temp'];
    end
    temp = times(end):Dtime:2*Tc;
    temp(end)=[];
    temp(1)=[];
    times=[times;temp'];
    Z=[Z;ones(length(temp),1)*lan];

    numCycle = round(t/2/Tc+0.5);
    oneCycle = times;
    oneZ = Z;
    for i=2:numCycle
        oneCycle = 2*Tc+oneCycle;
        temp = oneCycle;
        temp(1)=[];
        times=[times;temp];
        temp = oneZ;
        temp(1)=[];
        Z = [Z;temp];
    end
    timewalk = 0:Dtime:t;
    len = length(timewalk);
    times = times(1:len+1);
    Z = Z(1:len+1);
    Z_a = [times Z];
end

