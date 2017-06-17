function varargout = dhtable2T(dh)
% function T = dhtable2T(dh)
% function [T1,T2,...,TG] = dhtable2T(dh)
%
%   Computes the homogeneous matrix relating the reference frame attached
%   to the end effector with the fixed reference frame given the
%   Denavit-Hartenverg (D-H) parameters at each time slot. In other words,
%   it computes the homogeneous matrix representing the position and
%   orientation of the end effector related to the fixed reference frame.
%   Where:
%
%       dh: a G x 4 x N matrix of real numbers, being G the number of
%          d.o.f. of the serial robot and N the number of time slots.
%          Hence, dh(:,:,1) is the D-H table at t=0, dh(3,1,:) is the
%          time evolution of the theta parameter of the third joint, and
%          so on. Note dh(:,1,:) and dh(:,4,:) are respectively the
%          theta and alpha angles, therefore radians are assumed, while
%          dh(:,2,:) and dh(:,3,:) are respectively the d and a
%          distances, therefore length units are assumed.
%       T: A 4 x 4 x N array of real numbers representing the rigid 
%          transformation from the fixed reference frame to the end
%          effector at each time slot. T(:,:,n) is the 4 x 4 homogeneous
%          matrix representing the change in reference frame for t=tn.
%       T1, T2, ..., TG: when the second form is used, the number of output
%          arguments MUST match the number G of rows in the D-H table. In
%          this case, each successive T1, T2, ..., TG is a 4 x 4 x N array 
%          of real numbers representing the rigid motion from reference 0
%          to reference 1 (T1(:,:,n)), from reference 1 to reference 2 
%          (T2(:,:,n)), ..., from reference G-1 to reference G (TG(:,:,n))
%          at time slot t=tn. Reference i is attached to the i-th link of
%          the serial robot, so that reference 0 is the fixed reference
%          (ground) and reference G is the end effector. In this case, the
%          global rigid transformation (the T computed with the first form 
%          of the function) reads; T(:,:,n) = T1(:,:,n)*T2(:,:,n)* ...
%          *TG(:,:,n) for each t = tn.

%% --------------------------------------------------------------------------
% Sanity checks:
if(nargin~=1)
    error('Wrong number of input arguments');
end
if(ndims(dh)>3)
    error('dh should be either a Gx4 matrix or a Gx4xN 3D array');
end
[G,P,~] = size(dh);
if(P~=4)
    error('The D-H table MUST have 4 (and just 4) columns');
end
if( (nargout>1) && (nargout~=G) )
    error('The number of output arguments should either be just 1 or match the number G fo rows in dh');
end

%% --------------------------------------------------------------------------
% Everything seems to be OK. Compute all Aij natrixes:
Aij = computeAMatrixesFromDH(dh); % 4x4xGxN
% Compute their products if needed (if one argument is returned:
T   = [];
if(nargout==1)
    T = Aij(:,:,1,:); % 4x4x1xN
    for g=2:G
        T = rtarrayMatrixProduct( T, Aij(:,:,g,:) );
    end
    T = squeeze(T); % 4x4xN
end
%% --------------------------------------------------------------------------
% Return teh results
if(nargout==1)
    varargout{1} = T; % 4 x 4 x N
else
    varargout      = cell(1,G);
    for g=1:G
        varargout{g} = squeeze(Aij(:,:,g,:)); % 4x4xN each
    end
end

end

function Aij = computeAMatrixesFromDH(dh)
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
%% ------------
% Tranlation along z with parameter di
Atz(3,4,:,:) = reshape( dh(:,2,:), [1,1,G,N] );
%% ------------
% Tranlation along x with parameter ai
Atx(1,4,:,:) = reshape( dh(:,3,:), [1,1,G,N] );
%% ------------
% Rotation around x with parameter alpha
Arx(2,2,:,:) = reshape(  cos(dh(:,4,:)), [1,1,G,N] );
Arx(3,3,:,:) = reshape(  cos(dh(:,4,:)), [1,1,G,N] );
Arx(2,3,:,:) = reshape( -sin(dh(:,4,:)), [1,1,G,N] );
Arx(3,2,:,:) = reshape(  sin(dh(:,4,:)), [1,1,G,N] );
%% ------------
Aij = rtarrayMatrixProduct( Arz, Atz );   % 4x4xGxN
Aij = rtarrayMatrixProduct( Aij, Atx );   % 4x4xGxN
Aij = rtarrayMatrixProduct( Aij, Arx );   % 4x4xGxN
end




