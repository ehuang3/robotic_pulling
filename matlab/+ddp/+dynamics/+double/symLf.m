function [ Lf, Lfx, Lfxx ] = symLf( x, xf, p, k )
%SYMLF
%   

%%
xs = x(1:4);
xfs = xf(1:4);
ps = p(1:4);
ks = k(1:4);
d = x(5);
kd = k(5);
Lf = sum(ks.*(sqrt((xs-xfs).^2+ps.^2)-ps)) + kd*d^2;
Lfx = gradient(Lf,x);
Lfxx = hessian(Lf,x);

end

