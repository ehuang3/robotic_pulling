%% Zeros of trigonometric polynomials.

a = [1 2 3];
b = [1 2];

f = @(t,a,b) a(1) + a(2)*cos(t) + a(3)*cos(2*t) + b(1)*sin(t) + b(2)*sin(2*t);

FFM = [ 0 1 0 0; 0 0 1 0; 0 0 0 1; ...
       -1*(a(3)+i*b(2))/(a(3)-i*b(2)), ...
       -1*(a(2)+i*b(1))/(a(3)-i*b(2)), ...
       -1*(2*a(1))/(a(3)-i*b(2)), ...
       -1*(a(2)-i*b(1))/(a(3)-i*b(2))];

ev = eig(FFM);
t = angle(ev) - i*log(abs(ev));
f(t(1),a,b)
f(t(2),a,b)
f(t(3),a,b)
f(t(4),a,b)