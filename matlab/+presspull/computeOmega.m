function [ W, T ] = computeOmega( object, contact_point )
%COMPUTEOMEGA 
%   

%% 
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
cp0 = contact_point;

%% Translate contact point to origin and re-orient object.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
d = com0 - cp0;
t = -atan2(d(2),d(1)) - pi/2;
% cp = cp0 - cp0;
R = rot(t) * (R0 - repmat(cp0,1,size(R0,2)));

%% Compute omega bounds.
display('Computing omega...')
tic
W = zeros([1 length(T)]);
for i = 1:length(T)
    Rt = rot(T(i)) * R;
    Pt = object.P;
    xr = computeRotationCenter(Rt,Pt);
    W(i) = -1/xr;
end
toc

end

