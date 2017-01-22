%% Test compute angular velocities for a fixed pressure.
clc
clear
clear import

import data.*
import presspull.*

%% Load tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];
R = fillScanLines2DGrid2(K,X',Y',2e-2,true);

% Plot.
figure(1)
cla; hold on; grid on; axis equal;
plot(X,Y,'k')
plot(R(1,:),R(2,:),'b.')

tetra.V = [X';Y'];
tetra.K = K;
tetra.com = CoP;
tetra.R = R;
tetra.P = 1/size(R,2) * ones([1,size(R,2)]);

%% Compute omega.
[W,T] = computeOmega(tetra, [0;0]);