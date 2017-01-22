function [ P ] = generateCoMPressures( R, com, LB, UB )
%GENERATECOMPRESSURES Generate a feasible pressure distribution given the
%CoM.
%   R - surface tiling
%   com - center of mass
%   P - a feasible pressure distribution

%% Generate a feasible pressure distribution given the CoM.
% K = convhull(R');
% X = R(:,K);
X = R;

assert(size(com,1)==size(X,1),'Dimension mismatch');
assert(size(com,2)==1,'Point dimension error');

x_dim = size(X,2);

if nargin < 3
    LB = zeros([x_dim,1]);
    UB = ones([x_dim,1]);
end

f = zeros([x_dim,1]);
A = [];
b = [];
Aeq = [X; ones([1,x_dim])];
beq = [com; 1];
options = optimset('Display','none');
[P,~,flag,out] = linprog(f,A,b,Aeq,beq,LB,UB,[],options);
if flag < -2
    warning(out.message);
end
P = P';

end

