addpath Matfiles
addpath TienAnh
g = 9.81;
C_z = 0.4586;
StepLength = 0.3;
StepPeriod = 1;
HaftHipWidth=0.1;
EndTimes = 5.5;
DTime=0.01;
times = 0.5:DTime:EndTimes;
Cx_Ref = zeros(length(times),1);
Cy_Ref = zeros(length(times),1);
ddPx_Ref = zeros(length(times),1);
ddPy_Ref = zeros(length(times),1);
ZMPx_Ref = zeros(length(times),1);
ZMPy_Ref = zeros(length(times),1);
W_n = sqrt(g/C_z);
N=12;

for i=1:length(times)
    temp=0;
    for n=1:N
        Cy_Ref(i)=Cy_Ref(i)+2*HaftHipWidth*StepPeriod^2*W_n^2*...
            (1-cos(n*pi))*sin(n*pi*times(i)/StepPeriod)/...
            (n*pi*(StepPeriod^2*W_n^2+n^2*pi^2));
        
        temp=temp+StepLength*StepPeriod^2*W_n^2*(1+cos(n*pi))*...
            sin(n*pi*times(i)/StepPeriod)/(n*pi*(StepPeriod^2*W_n^2+...
            n^2*pi^2)); 
        % ZMP Refefence
        ddPx_Ref(i)=ddPx_Ref(i)-(n*pi/StepPeriod)^2*StepLength*StepPeriod^2*W_n^2*(1+cos(n*pi))*...
            sin(n*pi*times(i)/StepPeriod)/(n*pi*(StepPeriod^2*W_n^2+...
            n^2*pi^2));
        ddPy_Ref(i)=ddPy_Ref(i)-(n*pi/StepPeriod)^2*2*HaftHipWidth*StepPeriod^2*W_n^2*...
            (1-cos(n*pi))*sin(n*pi*times(i)/StepPeriod)/...
            (n*pi*(StepPeriod^2*W_n^2+n^2*pi^2));
    end
    Cx_Ref(i)=temp+StepLength*(times(i)-StepPeriod/2)/StepPeriod;
    ZMPx_Ref(i)=Cx_Ref(i)-C_z*ddPx_Ref(i)/g;
    ZMPy_Ref(i)=Cy_Ref(i)-C_z*ddPy_Ref(i)/g;
end

Px_Ref = zeros(length(times),1);
Py_Ref = zeros(length(times),1);
dcx1=0;
dcy1=0;
ddcx1=0;
ddcy1=0;
Px_Ref(1)=Cx_Ref(1);
Py_Ref(1)=Cy_Ref(1);
for i=2:length(times)
    dcx2=(Cx_Ref(i)-Cx_Ref(i-1))/DTime;
    dcy2=(Cy_Ref(i)-Cy_Ref(i-1))/DTime;
    ddcx2=(dcx2-dcx1)/DTime;
    ddcy2=(dcy2-dcy1)/DTime;
    Px_Ref(i)=Cx_Ref(i)-C_z*ddcx2/g;
    Py_Ref(i)=Cy_Ref(i)-C_z*ddcy2/g;
    ddcx1=ddcx2;
    ddcy1=ddcy2;
    dcx1=dcx2;
    dcy1=dcy2;
end
Cz_Ref = C_z*ones(length(times),1);

figure(1)
plot(times,[Cx_Ref, ZMPx_Ref]);
legend('Cx_R_e_f','ZMPx_R_e_f');
grid on

figure(2)
plot(times,[Cy_Ref, ZMPy_Ref]);
legend('Cy_R_e_f','ZMPy_R_e_f');
grid on
ZMPz_Ref=zeros(length(ZMPx_Ref),1);
ZMP_Ref=[ZMPx_Ref,ZMPy_Ref,ZMPz_Ref];
C_Ref=[Cx_Ref,Cy_Ref,Cz_Ref];

figure(3)
axis equal
view(3)
hold on
grid on
plot3(ZMP_Ref(:,1),ZMP_Ref(:,2),ZMP_Ref(:,3));
plot3(C_Ref(:,1),C_Ref(:,2),C_Ref(:,3));
tempx = xlim;
tempy = ylim;
tempz = zlim;
axis manual
axis([tempx*1.2 tempy*1.2 tempz*1.4])
daspect([1 1 1])
arrow3([tempx(1) 0 0],[tempx(2) 0 0],'-r1.0',1);
arrow3([0 tempy(1) 0],[0 tempy(2) 0],'-g1.0',1);
arrow3([0 0 tempz(1)],[0 0 tempz(2)*1.3],'-b1.0',1);
text(tempx(2),0,0,'X','FontWeight','bold','color','r');
text(0,tempy(2),0,'Y','FontWeight','bold','color','g');
text(0,0,tempz(2)*1.3,'Z','FontWeight','bold','color','b');
save('.\Matfiles\C_Ref.mat','C_Ref');
save('.\Matfiles\ZMP_Ref.mat','ZMP_Ref');