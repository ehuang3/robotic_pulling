function [ xnew, Fx, Fu ] = symF( x, u, ua, ub, la, lb, f )
%SYMF 
%   

%% 
import ddp.dynamics.double.symIDFT

w = x(3)-u(2)+pi;
alpha = symIDFT(w,ua,ub,f);
w = x(4)-u(2)+pi;
beta  = symIDFT(w,la,lb,f);

xnew(1) = x(1) + u(1)*cos(u(2));
xnew(2) = x(2) + u(1)*sin(u(2));
xnew(3) = x(3) + u(1)*alpha;
xnew(4) = x(4) + u(1)*beta;
xnew(5) = x(5) + u(1);
xnew = xnew';

Fx = jacobian(xnew, x);
Fu = jacobian(xnew, u);

end