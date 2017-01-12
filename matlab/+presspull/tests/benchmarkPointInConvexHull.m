%% Benchmark point in convex hull.
clc
clear

import presspull.*
import data.*

%% Load an object.
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
t = -atan2(d(2),d(1)) - pi/2;
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
t = [linspace(-pi,0) linspace(0,pi)];
B = zeros([2 length(t)]);
U = zeros([1 length(t)]);
L = zeros([1 length(t)]);
for i = 1:length(t)
    Vt = rot(t(i)) * V;
    comt = rot(t(i)) * com;
    Rt = rot(t(i)) * R;
    [xl, xu] = computeRotationCenterExtrema(Rt,comt(1),comt(2),Vt,K);
%     [xl, xu] = computeRotationCenterExtrema(Rt,comt(1),comt(2),Vt,K);
    U(i) = -1./xu;
    L(i) = -1./xl;
%     U(i) = xu;
%     L(i) = xl;
end
toc

%% Plot
figure(1)
subplot(121)
cla; hold on; axis auto; grid on;
plot(t,U,'r')
plot(t,L,'r')

%% Compute angular velocity bounds.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
display('computing bounds')
tic
t = [linspace(-pi,0) linspace(0,pi)];
B = zeros([2 length(t)]);
U = zeros([1 length(t)]);
L = zeros([1 length(t)]);
for i = 1:length(t)
    Vt = rot(t(i)) * V;
    comt = rot(t(i)) * com;
    Rt = rot(t(i)) * R;
    [wl, wu] = computeAngularVelocityBounds(Rt,comt(1),comt(2),1,Vt,K);
    U(i) = wu;
    L(i) = wl;
end
toc

%% Compute time to convergence.

tol = 2e-2;

subplot(122)
hold on; cla

Us = zeros([1 length(t)]);
Ls = zeros([1 length(t)]);
for i = 1:length(t)
    if abs(t(i)) < tol
        continue
    end
    odeFun = @(s,y) linterp(t,U,mod(y+pi,2*pi)-pi);
    eventFun = @(s,y) convergeEvent(s,y,sign(t(i))*tol,[]);
    opts=odeset('Events',eventFun);
    [s,~] = ode45(odeFun,[0,5],t(i),opts);
    Us(i) = max(s);
    odeFun = @(s,y) linterp(t,L,mod(y+pi,2*pi)-pi);
    [s,~] = ode45(odeFun,[0,5],t(i),opts);
    Ls(i) = max(s);
end

plot(t,Us,'k');
plot(t,Ls,'k');

%% 
w = linspace(0, 4*pi, 200);
w0 = mod(w+pi,2*pi)-pi;
subplot(211)
plot(w,w0)

%% 
[L,U,T] = computeBounds(obj,cp0);

%% 
V0 = obj.V;
com0 = obj.com;
r = max(sqrt(sum((V0-repmat(com0,[1,size(V0,2)])).^2)));
peshkin = obj;
s = linspace(0,2*pi);
peshkin.V = [r*cos(s)+com0(1); r*sin(s)+com0(2)];
peshkin.K = [(1:99)' (2:100)'];

[L,U,T] = computeBounds(peshkin,cp0)

%%
subplot(222)
hold on
plot(T,L,'b')
plot(T,U,'b')
grid on

%% rectangle

a = 0.06;
b = 0.025;
obj = rectangle(a,b);

V0 = obj.V;
K = obj.K;
com0 = obj.com;

n_cp = 30;
cps = getSpacedContactPoints(V0,K,n_cp);

r = max(sqrt(sum((V0-repmat(com0,[1,size(V0,2)])).^2)));

step = 1e-3;
R0 = fillScanLines2DGrid2(K,V0(1,:),V0(2,:),step);

%% Plot object, contact points and circumcircle.
X = [V0(1,K(:,1)); V0(1,K(:,2))];
Y = [V0(2,K(:,1)); V0(2,K(:,2))];
figure(1);
subplot(221)
cla
c_i = 5;
hold on; axis equal; grid on;
plot(X,Y,'b');
plot(R0(1,:),R0(2,:),'b.')
plot(cps(1,:),cps(2,:),'g.')
plot(cps(1,c_i),cps(2,c_i),'r.')
plot(com0(1),com0(2),'kx')
t = linspace(0,2*pi);
plot(r*cos(t)+com0(1),r*sin(t)+com0(2),'k')

%%
[L,U,T] = computeBounds(obj,cps(:,c_i));

%%
subplot(224)
hold on; grid on;
plot(T,L,'k')
plot(T,U,'k')

%%

tol = 20e-2;

subplot(223)
hold on; cla

Us = zeros([1 length(T)]);
Ls = zeros([1 length(T)]);
for i = 1:length(T)
    if abs(T(i)) < tol
        continue
    end
    odeFun = @(s,y) linterp(T,U,mod(y+pi,2*pi)-pi);
    eventFun = @(s,y) convergeEvent(s,y,sign(T(i))*tol,[]);
    opts=odeset('Events',eventFun);
    [s,~] = ode45(odeFun,[0,2],T(i),opts);
    Us(i) = max(s);
    odeFun = @(s,y) linterp(T,L,mod(y+pi,2*pi)-pi);
    [s,~] = ode45(odeFun,[0,2],T(i),opts);
    Ls(i) = max(s);
end

plot(T,Us,'k');
plot(T,Ls,'k');