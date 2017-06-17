global Tc Td Ds
xed = 0.3*Ds;
xsd = 0.3*Ds;
X_h = [];
time = [];
numCycle = 1;
Dtime = 0.001;
for k = 0:numCycle
    t = k*Tc:Dtime:k*Tc+Td;
    time=[time;t'];    
%     xh = k*Ds+(xed-xsd)/(Td^2*(Tc-Td))*((Td+k*Tc-t).^3-(Td+k*Tc-t)*Td^2-...
%         (t-k*Tc).^3+Td^2*(t-k*Tc))+xsd/Td*(Td+k*Tc-t)+xed/Td*(t-k*Tc);
    xh = k*Ds+(xed-xsd)/(Td^2*(Tc-Td))*((Td+k*Tc-t).^3-(Td+k*Tc-t)*Td^2-...
        (t-k*Tc).^3+Td^2*(t-k*Tc))+(Ds-xsd)/Td*(t-k*Tc)+xed/Td*(Td-t+k*Tc);
    X_h=[X_h;xh'];
    t = k*Tc+Td:Dtime:(k+1)*Tc;
%     t(1)=[];
%     if k~=numCycle
%         t(end)=[];
%     end
    time=[time;t'];
%     xh = k*Ds+(xsd+xed)/(Td*(Tc-Td)^2)*((Tc+k*Tc-t).^3-(Tc+k*Tc-t)*(Tc-Td)^2-...
%         (t-k*Tc-Td).^3+(Tc-Td)^2*(t-k*Tc-Td))+xed/(Tc-Td)*(Tc+k*Tc-t)+...
%         xsd/(Tc-Td)*(t-k*Tc-Td);
    xh = k*Ds+(Ds-xsd-xed)/(Td*(Tc-Td)^2)*((t-k*Tc-Td).^3-(Tc+k*Tc-t).^3-...
        (Tc+k*Tc-t)*(Tc-Td)^2-(Tc-Td)^2*(t-k*Tc-Td))+...
        (Ds+xed)/(Tc-Td)*(t-k*Tc-Td)+(Ds-xsd)/(Tc-Td)*(Tc+k*Tc-t);
    X_h = [X_h;xh'];    
    
end
plot(time,X_h)