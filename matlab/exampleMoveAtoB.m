%% Press-pull from A to B.
clear;
close all;
import presspull.*

%% Get rectangles A and B.
rect_A = [0, 0, pi/4,  0.5, 0.25]; % x, y, theta, l, w
rect_B = [1, 1, pi/1.33, 0.5, 0.25];
ug = 1.0;

% Contact points on A and B.
c = calcRectangleCorners([0,0,0,0.5,0.25]);
x_A = 0.5 * c(:,1);
x_B = 0.5 * c(:,2);
x_A_world = calcRectangleWorldCoord(x_A, rect_A);
x_B_world = calcRectangleWorldCoord(x_B, rect_B);

%% Plot rectangles and contact points.
hold on
plotRectangle(rect_A);
plotRectangle(rect_B);
plot(x_A_world(1),x_A_world(2),'k.');
plot(x_B_world(1),x_B_world(2),'k.');
axis equal
axis_lims = axis();

%% Line from x_B through COP_B
CoP_B = [rect_B(1); rect_B(2)];
l_B = CoP_B - x_B_world;
l_B = l_B / norm(l_B);

t = 5;
plot([x_B_world(1); x_B_world(1) + t*l_B(1)], ...
     [x_B_world(2); x_B_world(2) + t*l_B(2)], 'g');
axis(axis_lims);

%% Symbolic trigonometric polynomial.
t = sym('t', 'real');
d = sym('d', 'real');
gx = sym('gx', 'real');
gy = sym('gy', 'real');
s = sym('s', 'real');
k = sym('k', 'real');
ax = sym('ax', 'real');
ay = sym('ay', 'real');

G = [gx; gy];
A = [cos(t)*d + gx; sin(t)*d + gy];
X = [ax; ay];
AX = A - X;
BA = G - A;
perpBA = [BA(2); -BA(1)];
C = (1-t)*A + t*G  + k*perpBA;

Z = AX(1)*C(2) - AX(2)*C(1);
Z = collect(Z, [cos(t), sin(t)]);
Z = subs(Z, cos(t)^2, (1 + cos(2*t))/2);
Z = subs(Z, sin(t)^2, (1 - cos(2*t))/2);
Z = collect(Z, [cos(t), cos(2*t), sin(t)]);
co = coeffs(Z, [cos(t), cos(2*t), sin(t)]);

%% Compute A-B angle.
d = norm(x_A - x_B);
C = [0; 0];
BA = x_A - x_B;
BC = C - x_B;
t = sum(BC .* (BA)) / norm(BA)^2;
perpBA = [BA(2); -BA(1)];
k = sum((C - (t*x_B + (1-t)*x_A)).*perpBA) / norm(perpBA)^2;
ax = x_A(1);
ay = x_A(2);

CoP_B = [rect_B(1); rect_B(2)];
l_B = CoP_B - x_B_world;
l_B = l_B / norm(l_B);
G = x_B_world + ug * l_B;
gx = G(1);
gy = G(2);

i = 1;
circ = zeros([2,100]);
for tmp = linspace(0,2*pi)
    circ(1,i) = cos(tmp) * d + gx;
    circ(2,i) = sin(tmp) * d + gy;
    i = i + 1;
end
circ = [circ circ(:,1)];
plot(G(1),G(2),'k.')
plot(circ(1,:), circ(2,:), 'k')

a1 = k*d^2 + (gx*t - gx*(t - 1))*(ay - gy) - (gy*t - gy*(t - 1))*(ax - gx);
a2 = d*(ax - gx)*(t - 1) - d*(gx*t - gx*(t - 1)) - d*k*(ay - gy);
b0 = d*(gy*t - gy*(t - 1)) - d*(ay - gy)*(t - 1) - d*k*(ax - gx);

a = [0 a1 a2];
b = [b0 0];
theta = ccm(a,b);

%% Plot solution.
min_err = inf;
min_theta = 0;
for i = 1:length(theta)
    omega = real(theta(i));
    A = G + [cos(omega)*d; sin(omega)*d];
    BA = A - G;
    perpBA = [BA(2); -BA(1)];
    C = t*G + (1-t)*A + k * perpBA;
    AX = x_A_world - A;
    AC = C - A;
    err = abs(AX(1)*AC(2) - AX(2)*AC(1));
    if err < min_err
        err = min_err;
        min_theta = omega;
    end
end

A = G + [cos(min_theta)*d; sin(min_theta)*d];
BA = A - G;
perpBA = [BA(2); -BA(1)];
C = t*G + (1-t)*A + k * perpBA;

% Secant step refinement.
x(1) = min_theta + 0.05;
x(2) = min_theta;

A = G + [cos(x(1))*d; sin(x(1))*d];
BA = A - G;
perpBA = [BA(2); -BA(1)];
C = t*G + (1-t)*A + k * perpBA;
AX = x_A_world - A;
AC = C - A;
f(1) = abs(AX(1)*AC(2) - AX(2)*AC(1));

A = G + [cos(x(2))*d; sin(x(2))*d];
BA = A - G;
perpBA = [BA(2); -BA(1)];
C = t*G + (1-t)*A + k * perpBA;
AX = x_A_world - A;
AC = C - A;
f(2) = abs(AX(1)*AC(2) - AX(2)*AC(1));

for i = 3:10
    x(i) = x(i-1) - f(i-1)*(x(i-1)-x(i-2))/(f(i-1)-f(i-2));
    if abs(x(i) - x(i-1)) < 1e-7
        break
    end

    A = G + [cos(x(i))*d; sin(x(i))*d];
    BA = A - G;
    perpBA = [BA(2); -BA(1)];
    C = t*G + (1-t)*A + k * perpBA;
    AX = x_A_world - A;
    AC = C - A;
    f(i) = abs(AX(1)*AC(2) - AX(2)*AC(1));
end
min_theta = x(i);

%
plot(A(1),A(2),'b.');
plot(C(1),C(2),'r+');
plot([x_A_world(1) A(1)], [x_A_world(2) A(2)], 'g')

%% Rate of convergence.
tol = 1e-3;

CoP_A = [rect_A(1); rect_A(2)];
CoP_B = [rect_B(1); rect_B(2)];

t_1 = anglePointRay(CoP_A, x_A_world, A-x_A_world);
t_2 = anglePointRay(C, G, x_B_world-G);

a_1 = norm(CoP_A - x_A_world);
a_2 = norm(C - G);

swRate(a_1, t_1, tol)
norm(A-x_A_world)

swRate(a_2, t_2, tol)
norm(x_B_world - G)
