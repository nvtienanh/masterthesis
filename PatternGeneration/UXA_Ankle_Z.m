function [times,pos,vec] = UXA_Ankle_Z(height,tstart,tend,dt)
% Foot_Xa.m
% Function generation Foot_Xa
% Author: Nguyen Van Tien Anh
% Reference: Planning Walking Patterns for a Biped Robot
% Hi-tech Mechatronics Lab
% 08/01/2015
poly4th = @(coeff,x) coeff(1)*x.^4+coeff(2)*x.^3+coeff(3)*x.^2+...
        coeff(4)*x+coeff(5);   
dpoly4th = @(coeff,x) 4*coeff(1)*x.^3+3*coeff(2)*x.^2+2*coeff(3)*x+...
        coeff(4); 
    t1 = 0;    
    t3 = tend - tstart;
    t2 = (t1+t3)/2;
    f1 = 0;    
    f3 = 0;  
    f2 = height;
    A=[t1^4 t1^3 t1^2 t1 1;% f1
       t2^4 t2^3 t2^2 t2 1;% f2
       t3^4 t3^3 t3^2 t3 1;% f3       
       4*t1^3 3*t1^2 2*t1 1 0;% dx(strart) = 0
       4*t3^3 3*t3^2 2*t3 1 0];% dx(end) = 0
    Y = [f1;f2;f3;0;0];
    X = A\Y;
    
    times=(t1:dt:t3)';
    pos = poly4th(X,times);
    vec = dpoly4th(X,times);
    times = tstart+times;
end

