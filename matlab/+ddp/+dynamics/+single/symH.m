function [ H, Hx, Hxx, Hu, Hux, Huu ] = symH( x, u, lambda, wa, wb, f )
%SYMH 
%   

%% 
import ddp.dynamics.single.symL
import ddp.dynamics.single.symF

L = symL(x,u);
F = symF(x,u,wa,wb,f);

H = L + lambda*F;
Hx = gradient(H,x);
Hxx = hessian(H,x);
Hu = gradient(H,u);
Huu = hessian(H,u);
Huxux = hessian(H,[u x]);
Hux = Huxux(1:length(u),length(u)+1:end);

end

