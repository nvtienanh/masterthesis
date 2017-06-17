function BODY_p = ta_PositionBody(com_ref)
global uLINK
M = TotalMass(1);
BODY_p = (com_ref*M - ta_calcMC(1))/uLINK(1).m ...
    - uLINK(1).R * uLINK(1).c;
end