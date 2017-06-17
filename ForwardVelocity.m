function ForwardVelocity(j)
global uLINK

if j == 0 
    return; 
end
if j ~= 1
    mom = uLINK(j).mother;
%     uLINK(j).vel = uLINK(mom).vel +  uLINK(j).j*uLINK(j).dq;
%     uLINK(j).v = uLINK(j).vel(1:3);
%     uLINK(j).w = uLINK(j).vel(4:6);
    tmp2 = DHmod(uLINK(mom).DH,uLINK(mom).q); tmp2 = tmp2(1:3,4);
%     R = uLINK(mom).T(1:3,1:3);
    uLINK(j).v = uLINK(mom).v + cross(uLINK(mom).w, uLINK(mom).R*tmp2);
    uLINK(j).w = uLINK(mom).w + uLINK(j).T(1:3,3)* uLINK(j).dq;
    uLINK(j).vel = [uLINK(j).v;uLINK(j).w];


end
ForwardVelocity(uLINK(j).sister);
ForwardVelocity(uLINK(j).child);
