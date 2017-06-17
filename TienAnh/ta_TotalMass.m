function m = ta_TotalMass(j)
global uLINK

if j == 0
   m = 0;
else
   if j==1
       m = ta_TotalMass(uLINK(j).sister)...
       + ta_TotalMass(uLINK(j).child);
   else
       m = uLINK(j).m + ta_TotalMass(uLINK(j).sister)...
       + ta_TotalMass(uLINK(j).child);
   end
   
end