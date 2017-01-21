function [ Lf, Lfx, Lfxx ] = symLf( x, xf, p, k )
%SYMLF
%   

%%

Lf = sum(k.*(sqrt((x-xf).^2+p.^2)-p));
Lfx = gradient(Lf,x);
Lfxx = hessian(Lf,x);

end

