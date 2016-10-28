%% Center of pressure boundary example.
import presspull.*

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
plot(X(K)+0.5,Y(K),'b');
axis equal
grid on
hold on

% Compute rotation centers.
N = 100;
theta = linspace(pi/2,-pi/2,N);
x_r = zeros([N,1]);
m_f = zeros([N,1]);
for i = 1:length(theta)
    % Rotate tetrapod.
    rot = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
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
    x_r(i) = xr;
    m_f(i) = mf;

    % Compute G(R).
    f0 = 1;
    mu = 1;
    G_R = presspull.G(R,xr,-sign(xr),mu,f0);

    % Intersection with xy-plane.
    n = [0 0 1]';
    d = 0;
    [x0, y0, k0] = intersectPlaneConvexHull(n,d,R_x,R_y,G_R);

    % Plot.
    xy0 = inv(rot) * [x0';y0'];
    x0 = xy0(1,:);
    y0 = xy0(2,:);
    plot(x0(k0),y0(k0),'r');
    plot(0.5,0,'b*');
    pause(0.1);
end






