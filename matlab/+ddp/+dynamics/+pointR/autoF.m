function [xnew, Fx, Fu] = autoF(x, u, i, param)
x1 = x(1);
x2 = x(2);
u1 = u(1);
u2 = u(2);
xnew = [(x1 + u1*cos(u2)); (x2 + u1*sin(u2))];
Fx = [(1) (0); (0) (1)];
Fu = [(cos(u2)) (-u1*sin(u2)); (sin(u2)) (u1*cos(u2))];
end
