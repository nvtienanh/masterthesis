close all

clc


T=0.01;
N=length(ZMP_y);
g=9.8;
hcom=0.5;
 
A=[1 T (T^2)/2;0 1 T;0 0 1];
B=[(T^3)/6;(T^2)/2;T];
C=[1 0 -hcom/g];
 
PX=zeros(N,3);
x0=zeros(3,1);
zmpRef=ZMP_y;
% zmpRef(N/2:end)=0.1*ones(size(zmpRef(N/2:end)));
for i=1:N
PX(i,:) = C*A^i;
end
 
PU=zeros(N,N);
for i=1:N
    for j=1:i
        PU(i,j)=C*A^(i-j)*B;
    end
end
 
Q=10.0;
R=1E-6;
w=Q*PU'*PU+R*eye(N,N);
v=Q*PU'*PX*x0-Q*PU'*[zmpRef];
jerk=-inv(w)*v;
%now plot the trajectory
Xk=[];
Zk=[];
xk=x0;
for i=1:N
    xk=A*xk+B*jerk(i);
    zk=C*xk;
    Xk=[[Xk];xk'];
    Zk=[[Zk];zk'];
end
 
plot(Xk(:,1))
hold all
plot(zmpRef)
plot(Zk(:,1))
legend('COM','ZMP Ref','ZMP')