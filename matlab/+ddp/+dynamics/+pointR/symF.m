function [ xnew, Fx, Fu ] = symF( x, u )
%SYMF 
%   

%% 

xnew(1) = x(1) + u(1)*cos(u(2));
xnew(2) = x(2) + u(1)*sin(u(2));
xnew = xnew';

Fx = jacobian(xnew, x);
Fu = jacobian(xnew, u);

end