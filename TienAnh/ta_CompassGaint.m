close all
clear
clc
homedir = pwd;
matfiles_dir = sprintf('%s%sMatfiles',homedir,'\');
myfucn_dir = sprintf('%s%sTienAnh',homedir,'\');
addpath(matfiles_dir);
addpath(myfucn_dir);
load(sprintf('%s%sC_Ref.mat',matfiles_dir,'\'));
load(sprintf('%s%sFoot.mat',matfiles_dir,'\'));

global uLINK G M
G = 9.8;  % g [m/s^2]

SetupBipedRobot2;    % Figure 2.19, set the data of the two-legged walking 
%                     robot of Figure 2.20 (position of the center of 
%                     gravity, with the inertia tensor)
M = TotalMass(1);

%%%%%% Initial condition %%%%%
uLINK(BODY).p = [0.0, 0.0, 0.6]';
uLINK(BODY).R = eye(3);
uLINK(BODY).v = [0, 0, 0]';
uLINK(BODY).w = [0, 0, 0]';
idx = 1;
% Right Foot
Rfoot.p = [Foot(idx,3), -0.1, Foot(idx,5)]';
Rfoot.R = RPY2R([0,Foot(idx,7),0]);  %  -pi/4 < q < pi/4
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';

% Left Foot
Lfoot.p = [Foot(idx,2), 0.1, Foot(idx,4)]';
Lfoot.R = RPY2R([0,Foot(idx,6),0]); %  -pi/4 < q < pi/4
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';

% InverseKinematics
InverseKinematicsAll(RLEG_J5, Rfoot);
InverseKinematicsAll(LLEG_J5, Lfoot);
ForwardVelocity(1);

offset =[0.0025;-0.0340;-0.1218];
fig_1 = figure('Name','Compass Model');
movegui(fig_1,'northwest');
view([79 51])
grid on
hold on

% Foot= [times LeftX RightX LeftZ RightZ LeftTheta RightTheta]
time = Foot(:,1);
tsize = length(time);
Animation = VideoWriter('video.avi');
open(Animation);
for idx=1:tsize    
    Rfoot.p = [Foot(idx,3), -0.1, Foot(idx,5)]';
    Lfoot.p = [Foot(idx,2), 0.1, Foot(idx,4)]';
    temp = C_Ref(idx,:)'-offset;
    uLINK(BODY).p = temp;
        
    if idx ==1
        axis([0 1 -0.6 0.6 -0.2 1])
    else
        deltax = Foot(idx,3)-Foot(idx-1,3);        
    end
    plot3([Rfoot.p(1) uLINK(BODY).p(1)],[Rfoot.p(2) uLINK(BODY).p(2)],...
        [Rfoot.p(3) uLINK(BODY).p(3)],'-r');

    plot3([Lfoot.p(1) uLINK(BODY).p(1)],[Lfoot.p(2) uLINK(BODY).p(2)],...
        [Lfoot.p(3) uLINK(BODY).p(3)],'-b');
    pause(0.02);
    frame = getframe;
    writeVideo(Animation,frame);
end
close(Animation);