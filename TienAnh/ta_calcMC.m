function mc = ta_calcMC(j)
global uLINK

if j == 0
    mc = 0;
else    
    if j==1
        mc = 0;
    else
        mc = uLINK(j).m * (uLINK(j).p + uLINK(j).R * uLINK(j).c);
    end
    mc = mc + ta_calcMC(uLINK(j).sister) + ta_calcMC(uLINK(j).child);
end
