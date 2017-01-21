function [ H, Hx, Hxx, Hu, Hux, Huu ] = symH( x, u, lambda )
%SYMH 
%   

%% 
import ddp.*

L = dynamics.pointR.symL(x,u);
F = dynamics.pointR.symF(x,u);

H = L + lambda*F;
Hx = gradient(H,x);
Hxx = hessian(H,x);
Hu = gradient(H,u);
Huu = hessian(H,u);
Huxux = hessian(H,[u x]);
Hux = Huxux(1:length(u),length(u)+1:end);

end

