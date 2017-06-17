function [iter_taken, err, joints_vel_1, joints_vel_2] = ikine_sdls(to, Target)
% Selecty damped least square method
% http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
% See Equ. 11.
% r is the sructure of the robot.
% p (3x1) is the target cartesian position;
%
% THIS DOES NOT CHANGE STRUCTURE OF THE ROBOT!

global uLINK Dtime;
idx = FindRoute(to);

treshold = 1e-12;
max_iter = 100;
lambda_max  = 0.06;  %Variable Damping
beta        = 0.001;  %Isotropic Damping
epsilon     = 0.001; %Singular region size

iter_taken = 1;
dofs = length(idx);
nn = 0;
RJoints =[];
LJoints = [];
err = [];
joints_vel_1 =[];
joints_vel_2 =[];

while 1
    iter_taken = iter_taken + 1;  
    jac   = CalcJacobian(idx);
    delta_x = CalcVWerr(Target, uLINK(to));%p - x;    
    J = jac;
%% Damping Calculation   
% Maciejewski, A.A., Klein, C.A.: Numerical filtering for the operation  
% of robotic manipulators through kinematically singular configurations. 
% Journal of Field Robotics. 5, 527–552 (1988).
    [U,S,~] = svd(J);
    sigma_min = S(dofs,dofs);  
    u_m = U(:,dofs);
    lambda = lambda_max;
    if sigma_min < epsilon
        lambda = (1-(sigma_min/epsilon)^2)*(lambda_max^2);
    end
    %% Jacobian Inversion
    J_inv = (J')/(J*(J')+(lambda*lambda)*eye(dofs,dofs) + (beta*beta)*u_m*(u_m'));  
    dq = J_inv*delta_x;
    MoveJoints(idx, dq);
    ForwardKinematics(1); 
    
    % Get all joints angle in deg
    nn = nn+1;
    err(nn) = norm(delta_x);
    for k = 0:5    
        RJoints(nn,k+1) = 57.2958*uLINK(1+k).q;
        LJoints(nn,k+1) = 57.2958*uLINK(1+k).q;
    end
    
    if  iter_taken > max_iter
        %fprintf('**cgr_ikine2** breaks after %i iterations with errror %f.\n', iter_taken, err);
        break;
    end
%     if err < treshold || iter_taken > max_iter
%         %fprintf('**cgr_ikine2** breaks after %i iterations with errror %f.\n', iter_taken, err);
%         break;
%     end
end

for n=1:length(idx)
    j = idx(n);
%     uLINK(j).dq = vq(n);
%     uLINK(j).dq1 = vq1(n);
    uLINK(j).dq =  (uLINK(j).q - uLINK(j).q0);
    uLINK(j).q0 = uLINK(j).q; 
end


end