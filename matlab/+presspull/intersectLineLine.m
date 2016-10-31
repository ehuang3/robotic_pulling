function [ t, s ] = intersectLineLine( p, u, q, v )
%INTERSECTLINELINE
%   p - point on line 1
%   u - direction of line 1
%   q - point on line 2
%   v - direction of line 2

%% 
A = [u v];
b = q-p;
if rank(A) == 1
    t = inf;
    s = inf;
    return
end
x = A\b;
t = x(1);
s = -x(2);

end