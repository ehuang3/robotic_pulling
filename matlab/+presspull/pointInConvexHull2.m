function [ in_cvhull ] = pointInConvexHull2( p, X )
%POINTINCONVEXHULL2 
%   Linear programming version.

%% Test if point p is in the convex hull of X.

assert(size(p,1)==size(X,1),'Dimension mismatch');
assert(size(p,2)==1,'Point dimension error');

x_dim = size(X,2);
f = zeros([x_dim,1]);
A = [];
b = [];
Aeq = [X; ones([1,x_dim])];
beq = [p; 1];
LB = zeros([x_dim,1]);
UB = ones([x_dim,1]);
options = optimset('Display','none');
[~,~,flag,out] = linprog(f,A,b,Aeq,beq,LB,UB,[],options);
if flag < -2
    warning(out.message);
end
in_cvhull = (flag == 1);

end

