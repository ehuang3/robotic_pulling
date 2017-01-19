function [ F, Fx, Fu, Fval ] = symF( x, u, ua, ub, la, lb, f, dt)
%SYMF Symbolic discretized dynamics.
%   

%% Create symbolic dynamics.
import ddp.*

if ~isempty(x) && ~isempty(u)
    assert(length(x)==4,'Dimension mismatch');
    assert(length(u)==2,'Dimension mismatch');
end

vi = sym('v','real');
phi = sym('p','real');
xi = sym('x','real');
yi = sym('y','real');
ui = sym('u','real');
li = sym('l','real');
% di = sym('d','real');

omega = sym('w','real');
alpha = symIDFT(x, ua, ub, f);
alpha = subs(alpha,omega,ui+pi-phi);
beta = symIDFT(x, la, lb, f);
beta = subs(beta,omega,li+pi-phi);

fx = xi + vi*cos(phi)*dt;
fy = yi + vi*sin(phi)*dt;
fu = ui + vi*alpha*dt;
fl = li + vi*beta*dt;
% fd = di + vi*dt;

F = [fx fy fu fl]';
Fx = jacobian(F, [xi yi ui li]);
Fu = jacobian(F, [vi phi]);
if ~isempty(x) && ~isempty(u)
    Fval = double(subs(F,[xi yi ui li vi phi],[x(1) x(2) x(3) x(4) u(1) u(2)]));
else
    Fval = NaN;
end
% F = [fx fy fu fl fd]';
% Fval = double(subs(F,[xi yi ui li di vi phi],[x(1) x(2) x(3) x(4) x(5) u(1) u(2)]));


end

