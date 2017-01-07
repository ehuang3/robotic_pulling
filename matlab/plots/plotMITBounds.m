%% Plot angular velocity bounds.

clc
clear

import presspull.*
import data.*

%% Pick an object.
Objects = loadMITObjects;
keys = Objects.keys;
i = 7;
obj = Objects(keys{i})

V0 = obj.V;
K = obj.K;
com0 = obj.com;

%% Pick random contact point on the object boundary.
n_e = size(V0,2)-1;
r = randi(n_e);
e1 = V0(:,r);
e2 = V0(:,r+1);
s = rand
cp0 = s*e1 + (1-s)*e2;

%% Translate contact point to origin, orient object.
step = 3e-3;
Rg0 = fillScanLines2DGrid2(K,V0(1,:),V0(2,:),step);

% Shift contact point to origin and rotate object into 0 orientation.
n_v = size(V0,2);
d = com0 - cp0;
t = -atan2(d(2),d(1));
R = [cos(t) -sin(t); sin(t) cos(t)];
V = V0 - repmat(cp0,1,n_v);
V = R * V;
com = R * (com0 - cp0);
cp = cp0 - cp0;
Rg = R * (Rg0 - repmat(cp0,1,size(Rg0,2)));

%% Plot old and oriented object pose.
figure(2)
subplot(211)
hold on
cla;
axis equal
grid on
plot(V0(1,:),V0(2,:));
plot(com0(1),com0(2),'k*');
plot(cp0(1),cp0(2),'r*');
plot(Rg0(1,:),Rg0(2,:),'g.')
subplot(212)
hold on
cla;
axis equal
grid on
plot(V(1,:),V(2,:));
plot(com(1),com(2),'k*');
plot(cp(1),cp(2),'r*');
plot(Rg(1,:),Rg(2,:),'g.')

%% Compute angular velocity bound.
R = @(t) [cos(t) -sin(t); sin(t) cos(t)];

t = linspace(-pi/2,pi/2,200);
B = zeros([2 length(t)]);
U1 = zeros([1 length(t)]);
L1 = zeros([1 length(t)]);
for i = 1:length(t)
    Vt = R(t(i)) * V;
    comt = R(t(i)) * com;
    Rgt = R(t(i)) * Rg;
    [xl xu] = computeRotationCenterExtrema(Rgt,comt(1),comt(2),Vt,K);
    U1(i) = -1./xu;
    L1(i) = -1./xl;
%     B(1,i) = xu;
%     B(2,i) = xl;
end

%% Integrate angular velocity bounds.
TT = [(t-pi) t];
UU = [-U1 U1];
LL = [-L1 L1];

fun = @(t,y) linterp(TT,LL,y);
[tout, yout] = ode45(fun, [0 1], -pi/3)

%% Compute Peshkin's bound.

%% Plot moment hulls for min and max.

