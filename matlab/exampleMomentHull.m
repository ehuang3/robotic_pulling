%% Compute frictional moment hull.

import presspull.*

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
hold on
% plot(X,Y,'r')
% plot(CoP(1),CoP(2),'r*')
fill(X,Y,0.0*ones([1,3]))
% plot3(CoP(1),CoP(2),0,'r')

% Compute intertior points.
pts = fillScanLines2D(K,X,Y);
X = pts(:,1);
Y = pts(:,2);

% plot(pts(:,1),pts(:,2),'k.')
axis equal
grid on

% Compute G(R).
f0 = 1;
u = 1;
xr = 0.75;
G = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);
K = convhull(X,Y,G);
pointInConvexHull([CoP;0],X,Y,G)

% Plot G(R).
trimesh(K,X,Y,G,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
xlabel('x')
ylabel('y')
zlabel('g')
