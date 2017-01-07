%% Test point in convex hull.
import presspull.*

%% 3D Box.
Z = [0 0 0 0 1 1 1 1]';
X = [1 1 0 0 1 1 0 0]';
Y = [1 0 1 0 1 0 1 0]';
K = convhull(X,Y,Z);
trisurf(K,X,Y,Z,'facealpha',0.5)
axis vis3d

%% Linear programming version.
mu = [mean(X); mean(Y); mean(Z)];
A = [X'; Y'; Z'];
in_cvhull = pointInConvexHull3(mu,A)