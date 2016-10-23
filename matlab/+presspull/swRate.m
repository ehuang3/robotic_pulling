function [ dx ] = swRate( a, t, tol )
%SWRATE Silver watch rate of convergence.
%   

%% 
f = @(y) a.*atanh(sqrt(a.^2-y.^2)/a);
y = abs(sin(t) * a);
if cos(t) >= 0
    dx = f(y) + f(tol);
else
    dx = f(tol) - f(y);
end

end
