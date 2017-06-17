%%% Descriptions: Khoi tao Biped robot link
%% File: SetupBipedRobot.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
% BODY
uLINK    = struct('name','BODY','m',3.5,'sister',0,'child',2,'b',[0,0,0.527]','a',UZ,'q',0,'DH',[0, 0, 0, 0],'T',eye(4),'rot',rotz(90),'ub',pi/2,'lb',-pi/2);

% RIGHT LEG
uLINK(2) = struct('name','RLEG_J0','m',0.5,'sister',8,'child',3,'b',[0,-0.114/2,0]','a',UZ,'q',0,'DH',[-0.057, 0, -0.042, 0],'T',eye(4),'rot',rotz(90),'ub',pi/2,'lb',-pi/2);

uLINK(3) = struct('name','RLEG_J1','m',0.5,'sister',0,'child',4,'b',[0,0,-0.042]','a',UX,'q',0,'DH',[0, pi/2, 0 -pi/2],'T',eye(4),'rot',roty(90),'ub',pi/2,'lb',-pi/2);

uLINK(4) = struct('name','RLEG_J2','m',0.5,'sister',0,'child',5,'b',[0,0,0]','a',UY,'q',0,'DH',[0, -pi/2, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',-pi/2);

uLINK(5) = struct('name','RLEG_J3','m',0.5,'sister',0,'child',6,'b',[0,0,-0.21]','a',UY,'q',0,'DH',[0.21, 0, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',0);

uLINK(6) = struct('name','RLEG_J4','m',0.5,'sister',0,'child',7,'b',[0,0,-0.21]','a',UY,'q',0,'DH',[0.21, 0, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',-pi/2);

uLINK(7) = struct('name','RLEG_J5','m',0.5,'sister',0,'child',0,'b',[0,0,0]','a',UX,'q',0,'DH',[0, pi/2, 0 0],'T',eye(4),'rot',roty(90),'ub',pi/2,'lb',-pi/2);

% LEFT LEG
uLINK(8) = struct('name','LLEG_J0','m',0.5,'sister',0,'child',9,'b',[0,0.114/2,0]','a',UZ,'q',0,'DH',[0.057, 0, -0.042 0],'T',eye(4),'rot',rotz(90),'ub',pi/2,'lb',-pi/2);

uLINK(9) = struct('name','LLEG_J1','m',0.5,'sister',0,'child',10,'b',[0,0,-0.042]','a',UX,'q',0,'DH',[0, pi/2, 0 -pi/2],'T',eye(4),'rot',roty(90),'ub',pi/2,'lb',-pi/2);

uLINK(10)= struct('name','LLEG_J2','m',0.5,'sister',0,'child',11,'b',[0,0,0]','a',UY,'q',0,'DH',[0, -pi/2, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',-pi/2);

uLINK(11)= struct('name','LLEG_J3','m',0.5,'sister',0,'child',12,'b',[0,0,-0.21]','a',UY,'q',0,'DH',[0.21, 0, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',0);

uLINK(12)= struct('name','LLEG_J4','m',0.5,'sister',0,'child',13,'b',[0,0,-0.21]','a',UY,'q',0,'DH',[0.21, 0, 0 0],'T',eye(4),'rot',roty(90)*rotx(-90),'ub',pi/2,'lb',-pi/2);

uLINK(13)= struct('name','LLEG_J5','m',0.5,'sister',0,'child',0,'b',[0,0,0]','a',UX,'q',0,'DH',[0, pi/2, 0 0],'T',eye(4),'rot',roty(90),'ub',pi/2,'lb',-pi/2);

% Sister, to set the mother link based on the information of the daughter 
% link
FindMother(1); 

%% Thong so dong hoc cua robot
for n=1:length(uLINK)
    uLINK(n).q0     = 0;            % Inint joints position
    uLINK(n).dq     = 0;            % Joint velocity  [rad/s]
    uLINK(n).ddq    = 0;            % Joint acceleration [rad/s^2]
    uLINK(n).c      = [0 0 0]';     % Position of the center of gravity [m]
    uLINK(n).I      = zeros(3,3);   % Inertia tensor of the center of 
%                                     gravity around [kg.m^2]
    uLINK(n).Ir     = 0.0;          % Armature moment of inertia of the 
%                                     motor [kg.m^2]
    uLINK(n).gr     = 0.0;          % Reduction ratio of the motor [-]
    uLINK(n).u      = 0.0;          %  Joint torque [Nm]
    
    uLINK(n).vo = [0, 0, 0]';
    uLINK(n).v = [0, 0, 0]';
    uLINK(n).w  = [0, 0, 0]';
    uLINK(n).v1 = [0, 0, 0]';
    uLINK(n).w1  = [0, 0, 0]';
    uLINK(n).vel = [0, 0, 0, 0, 0, 0]';
    uLINK(n).j = [0, 0, 0, 0, 0, 0]';

    uLINK(n).dvo = [0, 0, 0]';
    uLINK(n).dw  = [0, 0, 0]';
end

%%% Make it easier to look at the program, and set the ID number to the
% link name with the same name as the variable
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

%%%%%%%%% Fuselage, and modeling rigid the foot %%%%%%%%%
shape = [0.08 0.16 0.52];     % Depth, width, height [m]
com   = [0 0 0.28]';        % Position of the center of gravity
SetupRigidBody(BODY, shape,com);
uLINK(BODY).c = rotz(-90)*uLINK(BODY).c;
uLINK(BODY).I = rotz(-90)*uLINK(BODY).I*rotz(-90)';
shape = [0.255 0.1 0.03];    % Depth, width, height [m]
com   = [0.0325  0 -0.055]';   % Position of the center of gravity
SetupRigidBody(RLEG_J5, shape,com);
uLINK(RLEG_J5).c = roty(-90)*uLINK(RLEG_J5).c;
uLINK(RLEG_J5).I = roty(-90)*uLINK(RLEG_J5).I*roty(-90)';
shape = [0.255 0.1 0.03];     % Depth, width, height [m]
com   = [0.0325  0 -0.055]';    % Position of the center of gravity
SetupRigidBody(LLEG_J5, shape,com);
uLINK(LLEG_J5).c = roty(-90)*uLINK(LLEG_J5).c;
uLINK(LLEG_J5).I = roty(-90)*uLINK(LLEG_J5).I*roty(-90)';
%%%%%%%%%%% Transition to non-attitude %%%%%%%%%%%%
uLINK(RLEG_J2).q = 0.0*ToRad; %-10
uLINK(RLEG_J3).q = 0.0*ToRad; %20
uLINK(RLEG_J4).q = 0.0*ToRad; %-10

uLINK(LLEG_J2).q = 0.0*ToRad; %-10
uLINK(LLEG_J3).q = 0.0*ToRad; %20
uLINK(LLEG_J4).q = 0.0*ToRad; %-10

uLINK(BODY).p = [0.0, 0.0, 0.462]';
uLINK(BODY).R = eye(3);
uLINK(BODY).T = [0 -1 0 0;
                     1 0 0 0;
                     0 0 1 0.49;
                     0 0 0 1];
ForwardKinematics(1);
