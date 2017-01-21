function [ xnew, Fx, Fu ] = symF( x, u, wa, wb, f )
%SYMF 
%   

%% 
import ddp.dynamics.double.symIDFT

w = x(3)-u(2)+pi;
omega = symIDFT(w,wa,wb,f);

xnew(1) = x(1) + u(1)*cos(u(2));
xnew(2) = x(2) + u(1)*sin(u(2));
xnew(3) = x(3) + u(1)*omega;
xnew = xnew';

Fx = jacobian(xnew, x);
Fu = jacobian(xnew, u);

end