%% Intersect 2D convex polygons.

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
plot(X(K)+0.5,Y(K),'b');
axis equal
grid on
hold on

theta = [pi/4, -pi/4];
x_r = [0 0];
m_f = [0 0];

% Rotate tetrapod.
rot = [cos(theta(1)) -sin(theta(1)); sin(theta(1)) cos(theta(1))];
X_ = X + 0.5;
Y_ = Y;
X_t = rot(1,1).*X_ + rot(1,2).*Y_;
Y_t = rot(2,1).*X_ + rot(2,2).*Y_;

% Fill scanlines.
R = fillScanLines2D(K,X_t,Y_t);
R_x = R(:,1);
R_y = R(:,2);
num_pts = size(R,1);
P = ones([num_pts,1]) ./ num_pts;

% Compute center.
[xr, mf] = computeRotationCenter(R,P);
x_r(1) = xr;
m_f(1) = mf;

% Compute G(R).
f0 = 1;
mu = 1;
G_R = presspull.G(R,xr,-sign(xr),mu,f0);

% Intersection with xy-plane.
n = [0 0 1]';
d = 0;
[x0, y0, k0] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);

% Rotate tetrapod.
rot = [cos(theta(2)) -sin(theta(2)); sin(theta(2)) cos(theta(2))];
X_ = X + 0.5;
Y_ = Y;
X_t = rot(1,1).*X_ + rot(1,2).*Y_;
Y_t = rot(2,1).*X_ + rot(2,2).*Y_;

% Fill scanlines.
R = fillScanLines2D(K,X_t,Y_t);
R_x = R(:,1);
R_y = R(:,2);
num_pts = size(R,1);
P = ones([num_pts,1]) ./ num_pts;

% Compute center.
[xr, mf] = computeRotationCenter(R,P);
x_r(2) = xr;
m_f(2) = mf;

% Compute G(R).
f0 = 1;
mu = 1;
G_R = presspull.G(R,xr,-sign(xr),mu,f0);

% Intersection with xy-plane.
n = [0 0 1]';
d = 0;
[x1, y1, k1] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);

plot(x0(k0),y0(k0),'r');
plot(x1(k1),y1(k1),'r');

x0 = x0(k0);
y0 = y0(k0);
x1 = x1(k1);
y1 = y1(k1);

%% 

% Find min/max in O(n) time.
[min0, i0] = min(y0);
[max0, j0] = max(y0);
[min1, i1] = min(y1);
[max1, j1] = max(y1);

% Compute intersection polygon.
[A,b] = computeConvexInteriorInequality2D(x0,y0);
for i = 1:length(x1)-1
    pt = [x1(i); y1(i)];
    all(A*pt >= b)
end

[A,b] = computeConvexInteriorInequality2D(x1,y1);
for i = 1:length(x0)-1
    pt = [x0(i); y0(i)];
    all(A*pt >= b)
end














