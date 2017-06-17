%%% SetupBipedRobot2.m
% Bipedal walking robot structure data: Figure 2.19, Figure 2.20 reference
% The definition of each field see "Table 2.1 link information"

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

uLINK    = struct('name','BODY'    , 'm', 10, 'sister', 0, 'child', 2, 'b',[0  0    0.65]','a',UZ,'q',0);

uLINK(2) = struct('name','RLEG_J0' , 'm',  5, 'sister', 8, 'child', 3, 'b',[0 -0.1 0]'   ,'a',UZ,'q',0);
uLINK(3) = struct('name','RLEG_J1' , 'm',  1, 'sister', 0, 'child', 4, 'b',[0  0   0]','a',UX,'q',0);
uLINK(4) = struct('name','RLEG_J2' , 'm',  5, 'sister', 0, 'child', 5, 'b',[0  0   0]'   ,'a',UY,'q',0);
uLINK(5) = struct('name','RLEG_J3' , 'm',  1, 'sister', 0, 'child', 6, 'b',[0  0  -0.3]' ,'a',UY,'q',0);
uLINK(6) = struct('name','RLEG_J4' , 'm',  6, 'sister', 0, 'child', 7, 'b',[0  0  -0.3]' ,'a',UY,'q',0);
uLINK(7) = struct('name','RLEG_J5' , 'm',  2, 'sister', 0, 'child', 0, 'b',[0  0   0.0]' ,'a',UX,'q',0);

uLINK(8) = struct('name','LLEG_J0' , 'm',  5, 'sister', 0, 'child', 9, 'b',[0  0.1 0]'   ,'a',UZ,'q',0);
uLINK(9) = struct('name','LLEG_J1' , 'm',  1, 'sister', 0, 'child',10, 'b',[0  0   0]','a',UX,'q',0);
uLINK(10)= struct('name','LLEG_J2' , 'm',  5, 'sister', 0, 'child',11, 'b',[0  0   0]'   ,'a',UY,'q',0);
uLINK(11)= struct('name','LLEG_J3' , 'm',  1, 'sister', 0, 'child',12, 'b',[0  0  -0.3]' ,'a',UY,'q',0);
uLINK(12)= struct('name','LLEG_J4' , 'm',  6, 'sister', 0, 'child',13, 'b',[0  0  -0.3]' ,'a',UY,'q',0);
uLINK(13)= struct('name','LLEG_J5' , 'm',  2, 'sister', 0, 'child', 0, 'b',[0  0   0]' ,'a',UX,'q',0);

% Sister, to set the mother link based on the information of the daughter 
% link
FindMother(1); 

%% Add Field
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % Joint velocity  [rad/s]
    uLINK(n).ddq    = 0;            % Joint acceleration [rad/s^2]
    uLINK(n).c      = [0 0 0]';     % Position of the center of gravity [m]
    uLINK(n).I      = zeros(3,3);   % Inertia tensor of the center of 
%                                     gravity around [kg.m^2]
    uLINK(n).Ir     = 0.0;          % Armature moment of inertia of the 
%                                     motor [kg.m^2]
    uLINK(n).gr     = 0.0;          % Reduction ratio of the motor [-]
    uLINK(n).u      = 0.0;          %  Joint torque [Nm]
end

%%% Make it easier to look at the program, and set the ID number to the
% link name with the same name as the variable
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

%%%%%%%%% Fuselage, and modeling rigid the foot %%%%%%%%%
shape = [0.1 0.3 0.5];     % Depth, width, height [m]
com   = [0 0 0.3]';        % Position of the center of gravity
SetupRigidBody(BODY,shape,com);

shape = [0.2 0.1 0.02];    % Depth, width, height [m]
com   = [0.03  0 -0.04]';   % Position of the center of gravity
SetupRigidBody(RLEG_J4, shape,com);

shape = [0.2 0.1 0.02];     % Depth, width, height [m]
com   = [0.03  0 -0.04]';    % Position of the center of gravity
SetupRigidBody(LLEG_J4, shape,com);

%%%%%%%%%%% Transition to non-attitude %%%%%%%%%%%%
uLINK(RLEG_J2).q = 0.0*ToRad; %-10
uLINK(RLEG_J3).q = 0.0*ToRad; %20
uLINK(RLEG_J4).q = 0.0*ToRad; %-10

uLINK(LLEG_J2).q = 0.0*ToRad; %-10
uLINK(LLEG_J3).q = 0.0*ToRad; %20
uLINK(LLEG_J4).q = 0.0*ToRad; %-10

uLINK(BODY).p = [0.0, 0.0, 0.5]';
uLINK(BODY).R = eye(3);

ForwardKinematics(1);
