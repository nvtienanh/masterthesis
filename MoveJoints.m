function MoveJoints(idx, dq)
global uLINK

for n=1:length(idx)
    j = idx(n);
    uLINK(j).q = uLINK(j).q + dq(n);
    uLINK(j).q  = min(uLINK(j).ub, max(uLINK(j).q , uLINK(j).lb)); % This is a saturation function.
end
