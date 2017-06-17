function T = DHmod(dh,qi)

% ai,api,qi,di,offset
%DH modified Summary of this function goes here
%   Detailed explanation goes here

% dhi = [ai,api,di,offset]
ai = dh(1);
api = dh(2);
di = dh(3);
offset = dh(4);

qi = qi + offset;
T = [cos(qi) -sin(qi) 0 ai;
    cos(api)*sin(qi) cos(api)*cos(qi) -sin(api) -di*sin(api);
    sin(api)*sin(qi) sin(api)*cos(qi) cos(api) di*cos(api);
    0 0 0 1];

end

