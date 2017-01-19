function [H,Hx,Hxx,Hu,Hux,Huu] = symH( x, u, ua, ub, la, lb, f, dt )
%SYMH 
%   H(x,u,lambda) = L(x,u) + lambda^T*F(x,u)
%   Hx
%   Hxx
%   Hu
%   Hux
%   Huu


%% 
import ddp.*

xi = sym('x','real');
yi = sym('y','real');
ui = sym('u','real');
li = sym('l','real');
vi = sym('v','real');
phi = sym('p','real');

[L, ~] = symL(x,u,dt);
[F, ~] = symF(x,u,ua,ub,la,lb,f,dt);
l1 = sym('l1','real');
l2 = sym('l2','real');
l3 = sym('l3','real');
l4 = sym('l4','real');
lambda = [l1 l2 l3 l4]';

H = L + lambda'*F;
Hx = gradient(H,[xi yi ui li]);
Hxx = hessian(H,[xi yi ui li]);
Hu = gradient(H,[vi phi]);
Huu = hessian(H,[vi phi]);
Huxux = hessian(H,[vi phi xi yi ui li]);
Hux = Huxux(1:2,3:6);

end

