function J = CalcJacobian(idx)
% Jacobian matrix of current configration in World frame
global uLINK
jsize = length(idx);
target = uLINK(idx(end)).p;   % absolute target position
J = zeros(6,jsize);
% TF2A = [0 0 -1 0.065;
%         0 1 0 0;
%         1 0 0 0;
%         0 0 0 1]; % Transform matrix from Foot coordinate to Ankle coordinate O_6
% target1 =  TF2A*[target;1];
% target1 = target1(1:3);

for n=1:jsize
    j = idx(n);
%      a = uLINK(j).R * uLINK(j).a ; % joint axis vector in world frame
    a = uLINK(j).T(1:3,3); 
    J(:,n) = [cross(a, target - uLINK(j).p) ; a ]; 
    uLINK(j).j = J(:,n);  
end

