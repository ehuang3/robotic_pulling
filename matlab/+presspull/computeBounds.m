function [ L, U, T ] = computeBounds( object, contact_point )
%COMPUTEBOUNDS Compute angular velocity bounds.
%   

%% Compute all angular velocity bounds for object.
import presspull.*

% Process inputs.
if nargin < 3
    T = linspace(-pi,pi,201);
    step = 2e-3;
end

%% Get object parameters.
V0 = object.V;
K = object.K;
com0 = object.com;
if isfield(object, 'R')
    R0 = object.R;
else
    R0 = fillScanLines2DGrid2(K,V0(1,:),V0(2,:),step);
end
if isfield(object,'LB') && isfield(object,'UB')
    LB = object.LB;
    UB = object.UB;
else
    LB = zeros([1,size(R0,2)]);
    UB = ones([1,size(R0,2)]);
end

cp0 = contact_point;

%% Translate contact point to origin and re-orient object.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];

n_v = size(V0,2);
d = com0 - cp0;
t = -atan2(d(2),d(1)) - pi/2;
V = V0 - repmat(cp0,1,n_v);
V = rot(t) * V;
com = rot(t) * (com0 - cp0);
% cp = cp0 - cp0;
R = rot(t) * (R0 - repmat(cp0,1,size(R0,2)));

%% Compute angular velocity bounds.
display('Computing bounds...')
tic
U = zeros([1 length(T)]);
L = zeros([1 length(T)]);
for i = 1:length(T)
    Vt = rot(T(i)) * V;
    comt = rot(T(i)) * com;
    Rt = rot(T(i)) * R;
    [wl, wu] = computeAngularVelocityBounds(Rt,comt(1),comt(2),1,Vt,K,LB,UB);
    U(i) = wu;
    L(i) = wl;
end
toc

end

