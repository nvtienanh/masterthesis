function com = ta_calcCoM()
global uLINK

M  = ta_TotalMass(1);
MC = ta_calcMC(1);
com = MC / M;