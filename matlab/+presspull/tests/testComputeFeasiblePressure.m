%% Test feasible pressure computation.
clc
clear

import presspull.*
import data.*

%% Load objects.
load('/home/eric/src/presspull/data/pods/3pod_0.10+0.025.mat')
i = 14;
obj = Objects(i)

%% Compute a feasible pressure distribution.
V = obj.V;
K = obj.K;
com = obj.com;
R = fillScanLines2DGrid2(K,V(1,:),V(2,:),3e-3);

figure(1);
subplot(211)
cla; hold on; grid on; axis equal;
x = [V(1,K(:,1)); V(1,K(:,2))];
y = [V(2,K(:,1)); V(2,K(:,2))];
plot(x,y,'b')
plot(R(1,:),R(2,:),'b.')
plot(com(1),com(2),'kx')

%%
K1 = convhull(V');
R1 = V(:,K1);
plot(V(1,K1),V(2,K1),'g')

%% 
X = V(:,K1);
p = com;

% 
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
[y,~,flag,out] = linprog(f,A,b,Aeq,beq,LB,UB,[],options);
if flag < -2
    warning(out.message);
end
in_cvhull = (flag == 1);
y = y';

%% 
computeRotationCenter(R1,y)



