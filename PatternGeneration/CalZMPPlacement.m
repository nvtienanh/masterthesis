function ZMPRef = CalZMPPlacement(ZMPDesignTable,ZMP_init,SwingFirst)
%CalZMPRef Function calculate ZMP reference from Walking table
%   Detailed explanation goes here
global Tcycle DSrate SSrate dt
zmp_x = ZMPDesignTable(1,:);
zmp_y = ZMPDesignTable(2,:);

[~,NumStep] = size(ZMPDesignTable);

if strcmp(SwingFirst,'Left')
    LeftStanceFirst = 0;
else
    LeftStanceFirst = 1;
end

ZMPRef = zeros(NumStep+1,2); % Stance Foot placement Desired
ZMPRef(1,1) = ZMP_init(1);
ZMPRef(1,2) = ZMP_init(2);

for n = 1:NumStep 
    % Equation 4.50    
    ZMPRef(n+1,1)     = ZMPRef(n,1) + zmp_x(n);
    if LeftStanceFirst        
        ZMPRef(n+1,2) = ZMPRef(n,2) - (-1)^(n)*zmp_y(n);
    else        
        ZMPRef(n+1,2) = ZMPRef(n,2) + (-1)^(n)*zmp_y(n);  
    end    
end


end

