%% Find the zero crossing of mf.

% Initial values.
mu = 1;
f0 = 1;
r_IC = [1;0]';
w = -1;
theta = pi/6;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.5;
Y = Y + 0.5;
X = rot(1,1).*X + rot(1,2).*Y;
Y = rot(2,1).*X + rot(2,2).*Y;
R = fillScanLines2D(K,X,Y);
num_pts = size(R,1);
R_x = R(:,1);
R_y = R(:,2);
P = ones([num_pts,1]) ./ num_pts;

% Compute the center of pressure.
CoP = R'*P;

% If the CoP lies in the negative x plane, flip it.
if CoP(1) < 0
    R(:,1) = -R(:,1);
end

% Bisection method.
xl = 0;
xu = 1;
tol = 1e-9;
l = computeFrictionalMoment(mu,xl,w,R,P);
u = computeFrictionalMoment(mu,xu,w,R,P);
mf = inf;
max_iters = 30;
i = 0;
while abs(mf) > tol && i < max_iters
    d = floor(abs(sign(l)-sign(u))/2);
    p = -(abs(sign(l)-sign(u))-1);
    xr = xu + (-1)^d*2^p*(xu-xl);
    mf = computeFrictionalMoment(mu,xr,w,R,P);
    if d == 0
        xl = xu;
        xu = xr;
        l = u;
        u = mf;
    elseif sign(l) == sign(mf)
        xl = xr;
        l = mf;
    else
        xu = xr;
        u = mf;
    end
    i = i + 1;
end

% If the CoP lies in the negative x plane, unflip the rotation center.
if CoP(1) < 0
    xr = -xr;
end

%% 
xr = computeRotationCenter(R,P);