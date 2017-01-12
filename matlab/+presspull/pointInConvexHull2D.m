function [ in_cvhull ] = pointInConvexHull2D( pt, X, Y )
%POINTINCONVEXHULL2D 
%   

%% 
import presspull.*
K = convhull(X,Y);
X = X(K);
Y = Y(K);
[A, b] = computeConvexInteriorInequality2D(X,Y);
in_cvhull = all(A*pt >= b);

end

