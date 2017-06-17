function J = dhtable2Jacobian(varargin)
% function J = dhtable2Jacobian(dh,jointtype[,A])
%
%   Computes the jacobian (partial derivatives matrix) of a serial robot,
%   which relates the end effector velocity with the joints velocities, at
%   each time slot, given the Denavit-Hartenberg (D-H) parameters.
%
%       dh: a G x 4 x N matrix of real numbers, being G the number of
%          d.o.f. of the serial robot and N the number of time slots.
%          Hence, dh(:,:,1) is the D-H table at t=0, dh(3,1,:) is the
%          time evolution of the theta parameter of the third joint, and
%          so on. Note dh(:,1,:) and dh(:,4,:) are respectively the
%          theta and alpha angles, therefore radians are assumed, while
%          dh(:,2,:) and dh(:,3,:) are respectively the d and a
%          distances, therefore length units are assumed.
%       jointype: a G x 1 (column) vector of scalars indicating wether the
%          correponding d.o.f. is a prismatic (0) or a revolution (~=0)
%          joint. For example, jointtype = [0,0,0,1,1,1]' corresponds to a
%          cartesian robot with a (possibly spherical) wrist, and jointtype
%          = [1,1,1,1,1,1]' corresponds to an antropomorphic robot with a
%          (possibly spherical) wrist.
%       A: this argument is optional, and, if provided, must have size
%          G x 1 (column vector). It is used to draw a linear relation 
%          between the actual joint value (displacement of the prismatic,
%          rotation of the revolute) and the value of the D-H parameter 
%          (d or theta). For example, if the thetai parameter of a certain 
%          joint reads thetai = qi + pi/2, you should set A(i)=1. If the dj
%          parameter of a different joint j reads dj = l0-2*qj, you should
%          set A(j)=-2, and so on.
%       J: a 6 x G x N array of real numbers with the Jacobian values. For
%          each tn, J(:,:,n) is the 6xG Jacobian matrix at t=tn, such that
%          V = J*[u1,u2,...,uG]', with u1,u2,...,uG the joint velocities,
%          is a 6 x 1 vector: V(1:3) is the linear velocity of the end
%          effector and V(4:6) is the angular velocity of the end effector,
%          both of them measured in the fixed reference frame (reference 0
%          attached to the ground).

%--------------------------------------------------------------------------
%% Parse inputs:
[dh,jointtype,A] = parse_inputs(varargin{:});
[G,~,N] = size(dh);
J = zeros(6,G,N);
%--------------------------------------------------------------------------
%% In this case we will use homogeneous matrixes algebra instead of
% dual quaternions for the sake of efficiency. Working with dual
% quaternions results in very long execution times, so we will use plain
% matlab arrays instead.
%--------------------------------------------------------------------------
%% Compute the A matrixes and their derivatives:
[Ad0,Ad1] = derivativesDegreesOfFreedom(dh,jointtype,A);
% Ad0 is 4x4xGxN, Ad0(:,:,g,:) is the hypermatrix with the reference change
% between g-1 and g. Ad1 is 4x4xGxN, Ad1(:,:;g,:) is the hypermatrix
% computed as \partial Ad0(:,:,g,:) / \partial q_g
cum = Ad0(:,:,G,:); % 4x4xN, cumulative matrix product
%% Backwards recursion: multiply each homogeneous matrix by the previous one
% to compute the derivative of the whole with respect to each joint
% variable:
for g=G-1:-1:1
    Ad1(:,:,g,:) = rtarrayMatrixProduct( Ad1(:,:,g,:), cum );
    cum          = rtarrayMatrixProduct( Ad0(:,:,g,:), cum );
end
%% Forward recursion: multiply each homogeneous matrix by the next one to
% compute the derivative and the position:
for g=2:G
    Ad1(:,:,g,:) = rtarrayMatrixProduct( Ad0(:,:,g-1,:), Ad1(:,:,g,:) );
    Ad0(:,:,g,:) = rtarrayMatrixProduct( Ad0(:,:,g-1,:), Ad0(:,:,g,:) );
end
%% LINEAR VELOCITIES
% Compute the first part of the jacobian (first three rows), corresponding
% to the linear velocity, i.e. to the derivatives of the fourth column of
% the successive homogeneous matrixes in the chain:
J(1:3,:,:) = reshape( Ad1(1:3,4,:,:), [3,G,N] ); % 3xGxN
%% ANGULAR VELOCITIES
% Compute the second part of the jacobian (last three rows), corresponding
% to the angular velocity. This angular velocity is the superposition of
% the rotation velocities of each revolution joint. Following the D-H
% convention, these joints rotate around axes z0, z1, ..., z{G-1}, so that
% each sub-column is just z0, z1...
J(4:6,2:G,:) = reshape( Ad0(1:3,3,1:G-1,:), [3,G-1,N] ); % The first G-1 z axes (corresponding to actual revolution joints)
J(6,1,:)     = 1; % The first joint revolution corresponds to the z0 axis
% But, when the joint is a prismatic one, the corresponding column must be
% set to 0 since the respectiver variable does not introduce any angular
% velocity:
J(4:6,~jointtype,:) = 0;
% Finally, the D-H parameters corresponding to rotation variables (thetai)
% might be linearly corrected, thetai = A(i)*q_i+B(i), so that we need a
% corresponding scaling of the column:
J(4:6,:,:) = J(4:6,:,:).*repmat(A',[3,1,N]);
%--------------------------------------------------------------------------
end

function [Ad0,Ad1] = derivativesDegreesOfFreedom(dh,jointtype,A)
[G,~,N] = size(dh);
%% ------------
Arz  = eye(4);                % 4x4
Arz  = repmat(Arz,[1,1,G,N]); % 4x4xGxN
Atz  = Arz;                   % 4x4xGxN, initalized to identity
Atx  = Arz;                   % 4x4xGxN, initalized to identity
Arx  = Arz;                   % 4x4xGxN, initalized to identity
%% ------------
% Rotation around z with parameter thetai
Arz(1,1,:,:) = reshape(  cos(dh(:,1,:)), [1,1,G,N] );
Arz(2,2,:,:) = reshape(  cos(dh(:,1,:)), [1,1,G,N] );
Arz(1,2,:,:) = reshape( -sin(dh(:,1,:)), [1,1,G,N] );
Arz(2,1,:,:) = reshape(  sin(dh(:,1,:)), [1,1,G,N] );
%---
% Its derivative (only for revolution joints):
nr   = length(find(jointtype));
Arzd = Arz;
Arzd(:,:,jointtype,:) = 0;
Arzd(1,1,jointtype,:) = reshape( -sin(dh(jointtype,1,:)), [1,1,nr,N] ) ...
    .*reshape( repmat(A(jointtype),[1,N]), [1,1,nr,N] ); % 1x1xGxN
Arzd(2,2,jointtype,:) = reshape( -sin(dh(jointtype,1,:)), [1,1,nr,N] ) ...
    .*reshape( repmat(A(jointtype),[1,N]), [1,1,nr,N] ); % 1x1xGxN
Arzd(1,2,jointtype,:) = reshape( -cos(dh(jointtype,1,:)), [1,1,nr,N] ) ...
    .*reshape( repmat(A(jointtype),[1,N]), [1,1,nr,N] ); % 1x1xGxN
Arzd(2,1,jointtype,:) = reshape(  cos(dh(jointtype,1,:)), [1,1,nr,N] ) ...
    .*reshape( repmat(A(jointtype),[1,N]), [1,1,nr,N] ); % 1x1xGxN
%% ------------
% Translation along z with parameter di
Atz(3,4,:,:) = reshape( dh(:,2,:), [1,1,G,N] );
%---
% Its derivative (only for prismatic joints):
nt   = length(find(~jointtype));
Atzd = Atz;
Atzd(:,:,~jointtype,:) = 0;
Atzd(3,4,~jointtype,:) = reshape( repmat(A(~jointtype),[1,N]), [1,1,nt,N] ); % 1x1xGxN
%% ------------
% Translation along x with parameter ai
Atx(1,4,:,:) = reshape( dh(:,3,:), [1,1,G,N] );
%% ------------
% Rotation around x with parameter alpha
Arx(2,2,:,:) = reshape(  cos(dh(:,4,:)), [1,1,G,N] );
Arx(3,3,:,:) = reshape(  cos(dh(:,4,:)), [1,1,G,N] );
Arx(2,3,:,:) = reshape( -sin(dh(:,4,:)), [1,1,G,N] );
Arx(3,2,:,:) = reshape(  sin(dh(:,4,:)), [1,1,G,N] );
%% ------------
Ad0 = rtarrayMatrixProduct( Arz, Atz );   % 4x4xGxN
Ad0 = rtarrayMatrixProduct( Ad0, Atx );   % 4x4xGxN
Ad0 = rtarrayMatrixProduct( Ad0, Arx );   % 4x4xGxN
%% ------------
Ad1 = rtarrayMatrixProduct( Arzd, Atzd ); % 4x4xGxN
Ad1 = rtarrayMatrixProduct( Ad1, Atx );   % 4x4xGxN
Ad1 = rtarrayMatrixProduct( Ad1, Arx );   % 4x4xGxN
%------------
end

function [dh,jointtype,A] = parse_inputs(varargin)
switch(nargin)
    case 0,
        error('Too few input arguments. At least the D-H table and the joint types are required');
    case 1,
        error('Too few input arguments. The joint types are mandatory');
    case 2,
        dh        = varargin{1};
        jointtype = varargin{2};
        A         = ones(size(jointtype));
    case 3,
        dh        = varargin{1};
        jointtype = varargin{2};
        A         = varargin{3};
    otherwise,
        error('Too many input arguments: 2 or 3 inputs are allowed');
end
%% --------------------------------------------------------------------------
% Sanity checks:
if(ndims(dh)>3)
    error('dh should be either a Gx4 matrix or a Gx4xN 3D array');
end
[G,P,~] = size(dh);
if(P~=4)
    error('The D-H table MUST have 4 (and just 4) columns');
end
if( (~ismatrix(jointtype)) || (size(jointtype,2)~=1) )
    error('The jointtype must be a column vector')
end
if(size(jointtype,1)~=G)
    error('The number of entries in jointtype must match the number G of rows (d.o.f.) in the D-H table');
end
jointtype = logical(jointtype);
if( (~ismatrix(A)) || (size(A,2)~=1) )
    error('A must be a column vector')
end
if(size(A,1)~=G)
    error('The number of entries in A must match the number G of rows (d.o.f.) in the D-H table');
end

end