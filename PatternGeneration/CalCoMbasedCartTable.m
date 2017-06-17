function [CoM,dCoM,ZMPout] = CalCoMbasedCartTable( ZMPRef,dt,hcom)
%CalCoMbasedZMP Function calculate CoM based on ZMP Ref
%   Detailed explanation goes here
N = length(ZMPRef);
g = 9.81;
 
A = [1 dt (dt^2)/2;0 1 dt;0 0 1];
B = [(dt^3)/6;(dt^2)/2;dt];
C = [1 0 -hcom/g];
 
PX=zeros(N,3);
x0=zeros(3,1);
y0=zeros(3,1);

for i=1:N
    PX(i,:) = C*A^i;
end
 
PU=zeros(N,N);
for i=1:N
    for j=1:i
        PU(i,j)=C*A^(i-j)*B;
    end
end
 
Q=1e5;
R=1;
w = Q*PU'*PU+R*eye(N,N);
v_x=Q*PU'*PX*x0-Q*PU'*ZMPRef(:,1);
v_y=Q*PU'*PX*x0-Q*PU'*ZMPRef(:,2);
jerk_x=-inv(w)*v_x;
jerk_y=-inv(w)*v_y;
%now plot the trajectory
Xk = [];
Yk = [];
Zk_x = [];
Zk_y = [];
xk = x0;
yk = y0;
for i=1:N
    %%% x direction
    xk=A*xk+B*jerk_x(i);
    zk=C*xk;
    Xk=[Xk;xk'];
    Zk_x=[Zk_x;zk'];
    %%% y direction
    yk=A*yk+B*jerk_y(i);
    zk=C*yk;
    Yk=[Yk;yk'];
    Zk_y=[Zk_y;zk'];
end
CoM    = [Xk(:,1) Yk(:,1)];
dCoM   = [Xk(:,2) Yk(:,2)];
ZMPout = [Zk_x Zk_y];
end

