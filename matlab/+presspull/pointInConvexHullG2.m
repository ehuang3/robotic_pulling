function [ in_cvhull ] = pointInConvexHullG2( pt, X, Y, Z, LB, UB )
%POINTINCONVEXHULLG2 
%   


%% 
import presspull.*

X = [X; Y; Z];
x_dim = size(X,2);

assert(size(pt,1)==size(X,1),'Dimension mismatch');
assert(size(pt,2)==1,'Point dimension error');

if nargin < 5
    LB = zeros([x_dim,1]);
    UB = ones([x_dim,1]);
end

f = zeros([x_dim,1]);
A = [];
b = [];
Aeq = [X; ones([1,x_dim])];
beq = [pt; 1];

% options = optimset('Display','none');
[x,~,flag] = glinprog(f,A,b,Aeq,beq,LB,UB);
in_cvhull = (flag > 0);

end

