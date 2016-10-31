%% Rotation centers of a presspulled body.

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);

% Compute rotation centers.
theta = linspace(pi/2,-pi/2);
x_r = [];
m_f = [];
for t = theta
    % Rotate tetrapod.
    rot = [cos(t) -sin(t); sin(t) cos(t)];
    X_ = X + 0.5;
    Y_ = Y;
    X_t = rot(1,1).*X_ + rot(1,2).*Y_;
    Y_t = rot(2,1).*X_ + rot(2,2).*Y_;
    plot(X_t(K), Y_t(K), 'r')
    grid on
    axis equal
    axis([-2 2 -2 2])

    % Fill scanlines.
    R = fillScanLines2D(K,X_t,Y_t);
    num_pts = size(R,1);
    P = ones([num_pts,1]) ./ num_pts;

    % Compute center.
    [xr, mf] = computeRotationCenter(R,P);
    x_r = [x_r xr];
    m_f = [m_f mf];

    pause(0.1)
end

%% 
plot(theta(2:end-1),x_r(2:end-1))