%% Plot angular velocity bounds for MIT Objects.
clc
clear

import presspull.*
import data.*

Objects = loadMITObjects;

%% Compute angular velocity bounds.
keys = Objects.keys;
i = keys(8);

% for i = Objects.keys

% Get object and its parameters.
obj = Objects(i{:});
V = obj.V;
K = obj.K;
com = obj.com;

% Pick random contact point on the object boundary.
n_e = size(V,2)-1;
r = randi(n_e);
e1 = V(:,r);
e2 = V(:,r+1);
s = rand;
cp = s*e1 + (1-s)*e2;

%% Plot object, COM, and contact point.
hold on
plot(V(1,:),V(2,:))
plot(com(1),com(2),'k*')
plot(cp(1),cp(2),'r.','MarkerSize',20)
grid on
axis equal

%% Compute exact angular velocity bound for given CoM.

% Shift contact point to origin and rotate object into 0 orientation.
n_v = size(V,2);
d = com - cp;
t = -atan2(d(2),d(1));
R = [cos(t) -sin(t); sin(t) cos(t)];
V1 = V - repmat(cp,1,n_v);
V1 = R * V1;
com1 = R * (com - cp);
cp1 = cp - cp;

%% Compute exact angular velocity bounds.
N = 10;
exact = zeros([2, N]);
t = linspace(0,2*pi,N);
for j = 1:N
    R = [cos(t(j)) -sin(t(j)); sin(t(j)) cos(t(j))];
    Vt = R * V1;
    Vt = fillScanLines2D(K, Vt(1,:), Vt(2,:), 1e-3);
    comt = R * com1;
    x0 = comt(1);
    y0 = comt(2);
    [wl, wu] = computeAngularVelocityBounds(Vt, x0, y0, 1);
    exact(1,j) = wu;
    exact(2,j) = wl;
end

% Compute Peshkin's angular velocity bound.

% Generate plot of distance-to-convergence.
% end

