function [ in_cvhull ] = pointInConvexHull3( p, X )
%POINTINCONVEXHULL3 
%   

%% Polar set version.

x = p - mean(X,2);
A = X - repmat(mean(X,2),[1 size(X,2)]);
A = A';

K = unique(convhull(A));
A = A(K,:);
size(A);

b = ones([size(A,1) 1]);
options = optimset('Display','none');
y = linprog(-x, A, b,[],[],[],[],[],options);
in_cvhull = (x'*y <= 1);

end

