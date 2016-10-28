function [ t, s ] = intersectLineLine( p, u, q, v )
%INTERSECTRAYRAY 
%   

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