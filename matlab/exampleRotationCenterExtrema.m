%% Compute rotation center extrema.
import presspull.*

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.5;

t = 0;
x0 = 0.5;
y0 = 0;

% Rotate tetrapod.
rot = [cos(t) -sin(t); sin(t) cos(t)];
X_t = rot(1,1).*X + rot(1,2).*Y;
Y_t = rot(2,1).*X + rot(2,2).*Y;

% Get region.
R = fillScanLines2D(K,X_t,Y_t);

% Get CoP.
CoP = [x0; y0];
CoP = rot*CoP;

% Compute extrema.
[xl, xu] = computeRotationCenterExtrema(R,CoP(1),CoP(2))

%% Plot CoP boundaries at extremas.

% Plot minima.
R_x = R(:,1);
R_y = R(:,2);
G_R = G(R, xl, -sign(xl), 1, 1);
n = [0 0 1]';
d = 0;
[x0, y0, k0] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);
figure(1);
hold on
plot(X_t(K),Y_t(K),'k');
plot(x0(k0),y0(k0),'r--');
plot(CoP(1),CoP(2),'b*');
axis equal
grid on

% Plot maxima.
R_x = R(:,1);
R_y = R(:,2);
G_R = G(R, xu, -sign(xu), 1, 1);
n = [0 0 1]';
d = 0;
[x1, y1, k1] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);
figure(2);
hold on
plot(X_t(K),Y_t(K),'k');
plot(x1(k1),y1(k1),'r--');
plot(CoP(1),CoP(2),'b*');
axis equal
grid on

%% Plot CoP boundaries between extrema.
close all;

for xr = linspace(xl, xu)
    % Plot boundary.
    R_x = R(:,1);
    R_y = R(:,2);
    G_R = G(R, xr, -sign(xr), 1, 1);
    n = [0 0 1]';
    d = 0;
    [x1, y1, k1] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);
    figure(3);
%     hold off
    plot(X_t(K),Y_t(K),'k');
    hold on
    plot(x1(k1),y1(k1),'r--');
    plot(CoP(1),CoP(2),'b*');
    title(['xr = ', num2str(xr)]);
    axis equal
    grid on
    pause(0.1)
end

%%
plot(xl,0,'k.','markersize',10)
plot(xu,0,'k.','markersize',10)