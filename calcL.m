function L = calcL(j)
global uLINK

if j == 0
   L = 0;
else
   c1 = uLINK(j).R*uLINK(j).c;   
   c  = uLINK(j).p + c1;   
   P = uLINK(j).m * (uLINK(j).v + cross(c1, uLINK(j).w));
   L = cross(c, P) + uLINK(j).R * uLINK(j).I * uLINK(j).R' * uLINK(j).w;
   L = L + calcL(uLINK(j).sister) + calcL(uLINK(j).child);
   %    display('------------')   
%    display(uLINK(j).R * uLINK(j).I * uLINK(j).R')
%    display(uLINK(j).T(1:3,1:3) * uLINK(j).I1 * uLINK(j).T(1:3,1:3)')
%  display('--------------------------')
%    display(uLINK(j).T(1:3,1:3) * uLINK(j).c1)
end