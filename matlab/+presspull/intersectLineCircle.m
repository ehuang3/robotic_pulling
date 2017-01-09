function [ t, w ] = intersectLineCircle( p, u, c, r )
%INTERSECTLINECIRCLE 
%   

%% Intersect line with circle.

C = zeros([1, 3]);
C(1) = u(1)^2 + u(2)^2;
C(2) = 2*u(1)*(p(1)-c(1)) + 2*u(2)*(p(2)-c(2));
C(3) = (p(1)-c(1))^2 + (p(2)-c(2))^2 - r^2;
t = roots(C);
w = zeros([2,1]);
x = [p(1) + t(1)*u(1), p(1) + t(2)*u(1)];
y = [p(2) + t(1)*u(2), p(2) + t(2)*u(2)];
w = atan2(y,x);
w(w<0) = w(w<0) + 2*pi;

end

