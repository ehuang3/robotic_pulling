%% Generate moment hull figures for the paper.
clear
close all;

import presspull.*

%% Plot tetrapod support region.
clf()

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];

% Rotate tetrapod.
theta = pi/4;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
XY = rot * [X';Y'];
X = XY(1,:)';
Y = XY(2,:)';
CoP = rot * CoP;

% Plot.
fill(X,Y,0.0*ones([1,3]))
% hold on
% plot(X,Y,'k')
% plot(CoP(1),CoP(2),'*b')
view([30,30])
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('g')
axis([-0.5 1 -0.75 1 -0.75 1])

% Export figure.
set(gcf, 'Color', 'w');
set(gca,'fontsize',14)
export_fig src/presspull/tex/fig/moment_hull_1.eps 

%% Plot G surface.
% Compute intertior points.
[XX,YY,I] = fillScanLines2DGrid(K,X,Y,1e-3);

% Compute G(R).
f0 = 1;
u = 1;
xr = 0.75;
% G_R = G(R,xr,-sign(xr),1);
G_R = u.*f0.*(XX.*XX + YY.*YY - XX.*xr)./sqrt((XX-xr).^2 + YY.^2);
G_R(~I) = NaN;
hold on
surf(XX,YY,G_R,'facealpha',0.5,'edgecolor','none')

% skip = 14;
rskip = round(linspace(1,size(XX,1),32));
cskip = round(linspace(1,size(XX,2),32));
surf(XX(rskip,:),YY(rskip,:),G_R(rskip,:),'FaceColor','none','MeshStyle','row');
surf(XX(:,cskip),YY(:,cskip),G_R(:,cskip),'FaceColor','none','MeshStyle','column');

axis([-0.5 1 -0.75 1 -0.75 1])

% Export figure.
set(gcf, 'Color', 'w');
set(gca,'fontsize',14)
export_fig src/presspull/tex/fig/moment_hull_2.eps 

%%
clf()

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];

% Rotate tetrapod.
theta = pi/4;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
XY = rot * [X';Y'];
X = XY(1,:)';
Y = XY(2,:)';
CoP = rot * CoP;

% Plot.
fill(X,Y,0.0*ones([1,3]))
% hold on
% plot(X,Y,'k')
% plot(CoP(1),CoP(2),'*b')
view([30,30])
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('g')
axis([-0.5 1 -0.75 1 -0.75 1])

%% Compute intertior points.
pts = fillScanLines2D(K,X,Y);
X = pts(:,1);
Y = pts(:,2);

G_R = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);

K = convhull(X,Y,G_R);
pointInConvexHull([CoP;0],X,Y,G_R)

% Plot G(R).
hold on
trimesh(K,X,Y,G_R,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
xlabel('x')
ylabel('y')
zlabel('g')
view([30,30])
axis equal
grid on
set(gca,'fontsize',14)
axis([-0.5 1 -0.75 1 -0.75 1])

% Export figure.
set(gcf, 'Color', 'w');
export_fig src/presspull/tex/fig/moment_hull_3.eps

%% Plot intersection.
clf()

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];

% Rotate tetrapod.
theta = pi/4;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
XY = rot * [X';Y'];
X = XY(1,:)';
Y = XY(2,:)';
CoP = rot * CoP;

% Plot.
plot(X,Y,'k')
hold on
plot(0,0,'k.','markersize',20)
plot(CoP(1),CoP(2),'b*')
axis equal
grid on

% Compute G(R)
pts = fillScanLines2D(K,X,Y);
X = pts(:,1);
Y = pts(:,2);
G_R = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);
K = convhull(X,Y,G_R);

% Intersect G(R) with xy-plane.
n = [0 0 1]';
d = 0;
[x0, y0, k0] = intersectPlaneConvexHull(n,d,X,Y,G_R);
plot(x0(k0),y0(k0),'r--');
axis([-0.5 1.2 -0.5 1.2])
set(gca,'ytick',[-0.5 0 0.5 1])
xlabel('x')
ylabel('y')
legend({'R','p_c','CoP','G(R)'})

% Export figure.
set(gca, 'Color', 'none');
set(gca,'fontsize',14)
export_fig src/presspull/tex/fig/CoP_boundary.eps -transparent
