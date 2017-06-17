%%% Descriptions: Demo giai dong hoc nguoc
%%% File: UXA_IK_demo.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc
addpath('myfunctions');
global uLINK       

SetupBipedRobot;   
TF2A = [0 0 -1 0.065;
        0 1 0 0;
        1 0 0 0;
        0 0 0 1]; % Transform matrix from Foot coordinate to Ankle coordinate O_6
%%%%%%%%%%% Vi tri ban dau cua robot%%%%%%%%%%%%
uLINK(RLEG_J2).q = 0.0*ToRad;
uLINK(RLEG_J3).q = 0.0*ToRad;
uLINK(RLEG_J4).q = 90.0*ToRad;
uLINK(RLEG_J5).q = 0.0*ToRad;

uLINK(LLEG_J2).q = 0.0*ToRad;
uLINK(LLEG_J3).q = 0.0*ToRad;
uLINK(LLEG_J4).q = 0.0*ToRad;

uLINK(BODY).T = [0 -1 0 0;
                     1 0 0 0;
                     0 0 1 0.527;
                     0 0 0 1];
ForwardKinematics(1); 

figure
DrawAllJoints(1);
view(10,20)
axis equal
zlim([-0.12 1.1]);
ylim([-0.15 .15]);
xlim([-0.1 .2]);
grid on
display('Press any key to solve inverse kinematic...')
pause

ForwardKinematics(1); 

% Calculate foot posture in world coordination
RFoot_T = uLINK(RLEG_J5).T*TF2A;
RFoot_eul = rotm2eul(RFoot_T(1:3,1:3)); % Euler angle of foot in world coordination
RFoot_pos = RFoot_T(1:3,4); % Position of foot in world coordination
cur_posture = [RFoot_pos' RFoot_eul];

ref_posture = [0.0, -0.057, 0.0, 0, 0, 0];% current posture of Foot

% REFERENCE POSTURE OF FOOT (IN WORLD COORDINATION)
Rfoot_p = [0.0, -0.057, 0.1]';
Rfoot_R = eye(3);  %  -pi/4 < q < pi/4
Rfoot.T = zeros(4,4);
Rfoot.T(1:3,1:3) = Rfoot_R;
Rfoot.T(1:4,4) = [Rfoot_p ;1];

Rfoot.T = Rfoot.T/TF2A;
Rfoot.p = Rfoot.T(1:3,4);
Rfoot.R = Rfoot.T(1:3,1:3);

err_posture = ref_posture - cur_posture;

to = RLEG_J5;
Target = Rfoot;

idx = FindRoute(to);

treshold = 1e-10;
max_iter = 70;
lambda_max  = 0.09;  %Variable Damping
beta        = 0.001;  %Isotropic Damping
epsilon     = 0.001; %Singular region size

iter_taken = 1;
dofs = length(idx);
nn = 0;
RJoints =[];
LJoints = [];
err = [];
 
for iter_taken=1:max_iter    
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
    % Calculate foot posture in world coordination
    RFoot_T = uLINK(RLEG_J5).T*TF2A;
    RFoot_eul = rotm2eul(RFoot_T(1:3,1:3)); % Euler angle of foot in world coordination
    RFoot_pos = RFoot_T(1:3,4); % Position of foot in world coordination
    cur_posture = [RFoot_pos' RFoot_eul];
%     eul = rotm2eul(uLINK(RLEG_J5).R);
%     posture = [uLINK(RLEG_J5).p' eul];
    err_posture = [err_posture;ref_posture - cur_posture];
%     clf
if mod(iter_taken,3)==0
    DrawAllJoints(1);
    view(10,20)
    axis equal
    zlim([-0.12 1.1]);
    ylim([-0.15 .15]);
    xlim([-0.1 .2]);
    grid on
    pause(0.05);
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
% clf
% DrawAllJoints(1);
%     view(10,20)
%     axis equal
%     zlim([-0.12 1.1]);
%     ylim([-0.15 .15]);
%     xlim([-0.1 .2]);
%     grid on
    
figure
subplot(3,2,1)
plot(err_posture(:,1),'-r')
xlim([0 70]);
grid on
subplot(3,2,3)
plot(err_posture(:,2),'-r')
xlim([0 70]);
grid on
subplot(3,2,5)
plot(err_posture(:,3),'-r')
xlim([0 70]);
grid on
subplot(3,2,2)
plot(err_posture(:,4)*57.2958,'-b')
xlim([0 70]);
grid on
subplot(3,2,4)
plot(err_posture(:,5)*57.2958,'-b')
xlim([0 70]);
grid on
subplot(3,2,6)
plot(err_posture(:,6)*57.2958,'-b')
xlim([0 70]);
grid on