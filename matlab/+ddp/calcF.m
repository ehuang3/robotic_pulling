function [ Fval ] = calcF( x, u, ua, ub, la, lb, f, dt )
%CALCF Compute dynamics.
%   

%% Compute dynamics.
import ddp.*

assert(length(x)==4,'Dimension mismatch');
assert(length(u)==2,'Dimension mismatch');

xi = x(1);
yi = x(2);
ui = x(3);
li = x(4);
vi = u(1);
phi = u(2);

alpha = idft(ui+pi-phi,ua,ub,f);
beta = idft(li+pi-phi,la,lb,f);

fx = xi + vi*cos(phi)*dt;
fy = yi + vi*sin(phi)*dt;
fu = ui + vi*alpha*dt;
fl = li + vi*beta*dt;

Fval = [fx fy fu fl]';

end

