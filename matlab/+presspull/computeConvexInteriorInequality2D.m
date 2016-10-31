function [ A, b ] = computeConvexInteriorInequality2D( X, Y )
%POINTINCONVEXHULL2D 
%   

%% 
N = length(X);
mu = [mean(X); mean(Y)];
A = zeros([N, 2]);
b = zeros([N, 1]);
for i = 1:N-1
    p1 = [X(i); Y(i)];
    p2 = [X(i+1); Y(i+1)];
    n = (p1-p2)./norm(p1-p2);
    n = [n(2); -n(1)];
    d = n'*p1;
    n = sign(n'*mu-d).*n;
    d = sign(n'*mu-d).*d;
    A(i,:) = n';
    b(i) = d;
end

end

