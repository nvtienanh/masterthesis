function [dh,jointtypes,A,B,jointmin,jointmax] = uxa_dhlleg
% function [dh,jointtypes,A,B,jointmin,jointmax] = snakerobot_geometry
%
%   Return the D-H geometrical configuration for a hyperedundant
%   snake robot:
%
%       dh: a 12 x 4 matrix of real numbers with the Denavit-Hartenberg
%          (D-H) table in either radians or meters. Entries
%          corresponding to d.o.f. (i.e. those non-constant) are filled
%          with zeros.
%       jointtypes: a 12 x 1 vectors of logicals where a 1 means the
%          corresponding joint is a revolute and 0 means the joint is a
%          prismatic (hence, this vector is all trues).
%       A, B: both of the are 12 x 1 vectors, and they are used to scale the
%          corresponding joint value to the proper D-H entry. If the i-th
%          joint is prismatic, then d_i = A(i)*q_i+B(i). If the i-th joint
%          is a revolute instead, them theta_i = A(i)*q_i+B(i) (note the
%          all joints are revolutes in the snake robot).
%       jointmin: a 12 x 1 vector with the minimum allowed value for each
%          joint (d.o.f.) in radians.
%       jointmax: a 12 x 1 vector with the maximum allowed value for each
%          joint (d.o.f.) in radians.
%
%   The D-H table used is as follows:

%  NEW
%      ------------------------------------------------------------
%      i       theta_i          d_i             a_i         alpha_i
%      ------------------------------------------------------------
%      1           q_1 + pi/2     0              0.085       - pi/2
%      2           q_2 + pi/2     0              0             pi/2
%      3           q_3            0              l2            0
%      4           q_4            0              l3            0
%      5           q_5            0              0             pi/2
%      6           q_6            0              l4            0
% Values in meter:
l1 = 0.042; % mm
l2 = 0.210; % mm
l3 = 0.210; % mm
l4 = 0.065; % mm
DOF = 6;
%---------------------------------------------
jointtypes = true(DOF,1);
jointmin   = -(pi)*ones(DOF,1);
jointmax   =  (pi)*ones(DOF,1);
A          = ones(DOF,1);
B          = zeros(DOF,1);
B(1) = pi/2;
B(2) = pi/2;
dh         = zeros(DOF,4);
d_i = [-l1 0 0 0 0 0]';
a_i = [0 0 l2 l3 0 l4]';
alpha_i = [-pi/2 pi/2 0 0 pi/2 0]';
dh(:,2) = d_i;
dh(:,3) = a_i;
dh(:,4) = alpha_i;



