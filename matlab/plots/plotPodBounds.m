%% Plot bounds and comparison.
clc
clear

import presspull.*
import data.*

%% Load objects.
load('/home/eric/src/presspull/data/pods/3pod_0.10+0.025.mat')
Objects

%% Pick an object and get contact points.
i = 1;
obj = Objects(i);
V0 = obj.V;
K = obj.K;
com0 = obj.com;

n_cp = 30;
cps = getSpacedContactPoints(V0,K,n_cp);

r = max(sqrt(sum((V0-repmat(com0,[1,size(V0,2)])).^2)));

step = 2e-3;
R0 = fillScanLines2DGrid2(K,V0(1,:),V0(2,:),step);

% Plot object, contact points and circumcircle.
X = [V0(1,K(:,1)); V0(1,K(:,2))];
Y = [V0(2,K(:,1)); V0(2,K(:,2))];
figure(1);
subplot(221)
cla
hold on; axis equal; grid on;
plot(X,Y,'b');
plot(R0(1,:),R0(2,:),'b.')
plot(cps(1,:),cps(2,:),'g.')
plot(com0(1),com0(2),'kx')
t = linspace(0,2*pi);
plot(r*cos(t)+com0(1),r*sin(t)+com0(2),'k')

% Compute percentage area.
p = obj.area / (pi*r^2)

%% Pick a contact point.
j = 3;
cp0 = cps(:,j);

rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];

% Translate contact point to origin and re-orient object.
n_v = size(V0,2);
d = com0 - cp0;
t = -atan2(d(2),d(1));
V = V0 - repmat(cp0,1,n_v);
V = rot(t) * V;
com = rot(t) * (com0 - cp0);
cp = cp0 - cp0;
R = rot(t) * (R0 - repmat(cp0,1,size(R0,2)));

% Plot zeroed object.
X = [V(1,K(:,1)); V(1,K(:,2))];
Y = [V(2,K(:,1)); V(2,K(:,2))];
subplot(223)
cla
hold on; axis equal; grid on;
plot(X,Y,'b');
plot(R(1,:),R(2,:),'b.');
plot(com(1),com(2),'kx');
plot(0,0,'ro')
t = linspace(0,2*pi);
plot(r*cos(t)+com(1),r*sin(t)+com(2),'k')

%% Compute angular velocity bounds.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
display('computing bounds')
tic
t = linspace(-3*pi/2,pi/2,200);
B = zeros([2 length(t)]);
U = zeros([1 length(t)]);
L = zeros([1 length(t)]);
for i = 1:length(t)
    Vt = rot(t(i)) * V;
    comt = rot(t(i)) * com;
    Rt = rot(t(i)) * R;
    [xl, xu] = computeRotationCenterExtrema(Rt,comt(1),comt(2),Vt,K);
    U(i) = -1./xu;
    L(i) = -1./xl;
end
toc

%% plot
subplot(224)
cla
hold on; axis square; grid on;
plot(t,U,'r');
plot(t,U,'g.');
plot(t,L,'b');
plot(t,L,'c.');

%% Compute distance-to-convergence.

t = linspace(-pi/2,pi/2,100);
TT = [linspace(-3*pi/2,-pi/2,100) t];
UU = [-fliplr(U) U];
LL = [-fliplr(L) L];

subplot(222)
cla
hold on; axis tight; grid on;
plot(TT,UU,'r');
plot(TT,UU,'g.');
plot(TT,LL,'b');
plot(TT,LL,'c.');

%% 
s_t = s_u
s = linspace(s_t-0.10,s_t+0.10,50);
for i = 1:length(s)
    Vs = rot(s(i)) * V;
    coms = rot(s(i)) * com;
    Rs = rot(s(i)) * R;
    [xl, xu] = computeRotationCenterExtrema(Rs,coms(1),coms(2),Vs,K);
    Us(i) = -1./xu;
    Ls(i) = -1./xl;
end

%% 
[~,i_b] = max(Ls);
s_u = s(i_b);
s_l = s(i_b-1);

figure(2)
subplot(211)
cla
hold on; axis equal; grid on;
R_u = rot(s_u) * R;
com_u = rot(s_u) * com;
plot(R_u(1,:),R_u(2,:),'b.')
plot(com_u(1),com_u(2),'kx')

subplot(212)
cla
hold on; axis equal; grid on;
R_l = rot(s_l) * R;
com_l = rot(s_l) * com;
plot(R_l(1,:),R_l(2,:),'b.')
plot(com_l(1),com_l(2),'kx')

%% plot
subplot(224)
cla
hold on; axis square; grid on;
plot(s,Us,'r');
plot(s,Us,'g.');
plot(s,Ls,'b');
plot(s,Ls,'c.');

%% Plot s_u
x_u = -1/s_u;
V_u = rot(s_u) * V;
com_u = rot(s_u) * com;
R_u = rot(s_u) * R;

% Plot object.
figure(1);
plot(R_u(1,:),R_u(2,:),'k.')
plot(com_u(1),com_u(2),'r.')
view([30,30])
hold on
grid on
axis auto
xlabel('x')
ylabel('y')
zlabel('g')

[G, X, Y] = calcG2(R_u, x_u, sign(x_u), 1, 1, V_u, K);
K_u = convhull(X,Y,G);
trimesh(K_u,X,Y,G,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
axis equal
title('s_u')

pointInConvexHull([com_u; 0], X, Y, G)

%% Plot s_l
x_l = -1/s_l;
V_l = rot(s_l) * V;
com_l = rot(s_l) * com;
R_l = rot(s_l) * R;

% Plot object.
figure(2)
hold on
plot(R_l(1,:),R_l(2,:),'k.')
plot(com_l(1),com_l(2),'r.')
view([30,30])
hold on
grid on
axis auto
xlabel('x')
ylabel('y')
zlabel('g')

[G, X, Y] = calcG2(R_l, x_l, sign(x_l), 1, 1, V_l, K);
K_l = convhull(X,Y,G);
trimesh(K_l,X,Y,G,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
axis equal
title('s_l')


%% Plot moment hull.
G_R = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);

K_u = convhull(X,Y,G_R);

hold on
trimesh(K_u,X,Y,G_R,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
xlabel('x')


%%
odefun = @(s,y) linterp(T,);





