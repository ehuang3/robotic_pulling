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
in_cvhull = pointInConvexHull2(mu,A)

%% Plane equations.
mu = [mean(X) mean(Y) mean(Z)]';
XYZ = [X Y Z];
A = zeros([size(K,1) 3]);
b = zeros([size(K,1) 1]);
for i = 1:size(K,1)
    p1 = XYZ(K(i,1),:)';
    p2 = XYZ(K(i,2),:)';
    p3 = XYZ(K(i,3),:)';
    P = [p1 p2 p3] - [p1 p1 p1];
    T = orth(P);
    n = cross(T(:,1),T(:,2));
    n0 = n / norm(n);
    d = p1'*n0;
    n0 = sign(mu'*n0 - d) .* n0;
    d = sign(mu'*n0 - d) .* d;
    A(i,:) = n0';
    b(i) = d;
end
in_cvhull = all(A*[0 0 -1]' >= b);