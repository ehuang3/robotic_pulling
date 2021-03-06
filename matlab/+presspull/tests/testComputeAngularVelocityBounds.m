%% Test angular velocity bounds.
% clc
clear

import presspull.*

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.5;
V = [X';Y'];

t = pi/3;
x0 = 0.5;
y0 = 0;

% Rotate tetrapod.
rot = [cos(t) -sin(t); sin(t) cos(t)];
X_t = rot(1,1).*X + rot(1,2).*Y;
Y_t = rot(2,1).*X + rot(2,2).*Y;

% Get region.
R = fillScanLines2D(K,X_t,Y_t);

% Get CoP.
CoP = [x0; y0];
CoP = rot*CoP;
x0 = CoP(1);
y0 = CoP(2);

% Create pressure bounds.
LB = zeros([1,size(R,2)]);
UB = 0.000125*ones([1,size(R,2)]);
% UB = ones([1,size(R,2)]);

[wl wu] = computeAngularVelocityBounds(R, x0, y0, 1, V, K, LB, UB)


