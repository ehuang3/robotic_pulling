function [ H, Hx, Hxx, Hu, Hux, Huu ] = symH( x, u, lambda, ua, ub, la, lb, f )
%SYMH 
%   

%% 
import ddp.dynamics.double.symL
import ddp.dynamics.double.symF

L = symL(x,u);
F = symF(x,u,ua,ub,la,lb,f);

H = L + lambda*F;
Hx = gradient(H,x);
Hxx = hessian(H,x);
Hu = gradient(H,u);
Huu = hessian(H,u);
Huxux = hessian(H,[u x]);
Hux = Huxux(1:length(u),length(u)+1:end);

end

