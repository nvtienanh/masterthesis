function ForwardKinematics(j)
global uLINK

if j == 0 
    return; 
end

if j ==1    
    uLINK(j).p = uLINK(j).T(1:3,4);
    uLINK(j).R = uLINK(j).T(1:3,1:3);
end
if j ~= 1    
    mom = uLINK(j).mother;
    uLINK(j).T = uLINK(mom).T*DHmod(uLINK(j).DH,uLINK(j).q);   
    uLINK(j).p = uLINK(j).T(1:3,4);
    uLINK(j).R = uLINK(j).T(1:3,1:3);
    
%     uLINK(j).p = uLINK(mom).R * uLINK(j).b + uLINK(mom).p;
%     uLINK(j).R = uLINK(mom).R * Rodrigues(uLINK(j).a, uLINK(j).q); 
end
% j = 1
ForwardKinematics(uLINK(j).sister);
ForwardKinematics(uLINK(j).child);
