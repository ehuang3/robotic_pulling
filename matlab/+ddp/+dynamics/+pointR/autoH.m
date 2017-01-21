function [H, Hx, Hxx, Hu, Hux, Huu] = autoH(x, u, lambda, i, param)
x1 = x(1);
x2 = x(2);
u1 = u(1);
u2 = u(2);
l1 = lambda(1);
l2 = lambda(2);
H = [(l1*(x1 + u1*cos(u2)) + l2*(x2 + u1*sin(u2)))];
Hx = [(l1); (l2)];
Hxx = [(0) (0); (0) (0)];
Hu = [(l1*cos(u2) + l2*sin(u2)); (l2*u1*cos(u2) - l1*u1*sin(u2))];
Hux = [(0) (0); (0) (0)];
Huu = [(0) (l2*cos(u2) - l1*sin(u2)); (l2*cos(u2) - l1*sin(u2)) (- l1*u1*cos(u2) - l2*u1*sin(u2))];
end
