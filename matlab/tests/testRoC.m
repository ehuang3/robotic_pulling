%% Rate of Convergence.
clear;
close all;

a = 0.1;
y_d = 1e-3;

f = @(y) -sqrt(a.^2-y.^2) + a.*atanh(sqrt(a.^2-y.^2)/a);
df = @(y) -y./sqrt(a.^2-y.^2);

y = linspace(0.1,1e-3);
f(y)
hold on
plot(f(y) + sqrt(a.^2-y.^2),y)
plot(-f(y)- sqrt(a.^2-y.^2),y)
axis equal
grid on

%%

a = 0.1;
t = pi/4;
tol = 1e-3;

f = @(y) a.*atanh(sqrt(a.^2-y.^2)/a);
y = sin(t) * a;
if cos(t) >= 0
    dx = f(y) + f(tol);
else
    dx = f(tol) - f(y);
end

