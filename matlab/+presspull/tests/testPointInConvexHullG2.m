%% Test Gurobi 2 example
clc
clear

import presspull.*

%% 3D Box.
Z = [0 0 0 0 1 1 1 1];
X = [1 1 0 0 1 1 0 0];
Y = [1 0 1 0 1 0 1 0];
K = convhull(X,Y,Z);
% trisurf(K,X,Y,Z,'facealpha',0.5)
% axis vis3d

%%
mu = [mean(X); mean(Y); mean(Z)];
mu = rand([3,1])-0.5
LB = 0.1 * ones(size(Z));
UB = ones(size(Z));
in_cvhull = pointInConvexHullG2(mu,X,Y,Z,LB,UB)
