function [ in_cvhull ] = inConvexHull( pt, X, Y, Z )
%INCONVEXHULL 
%   

%% 
mu = [mean(X) mean(Y) mean(Z)]';
XYZ = [X Y Z];
A = zeros([size(K,1) 3]);
b = zeros([size(K,1) 1]);
for i = 1:size(K,1)
    p1 = XYZ(K(i,1),:)';
    p2 = XYZ(K(i,2),:)';
    p3 = XYZ(K(i,3),:)';
    P = [p1 p2 p3] - [p1 p1 p1];
    T = orth(P);
    n = cross(T(:,1),T(:,2));
    n0 = n / norm(n);
    d = p1'*n0;
    n0 = sign(mu'*n0 - d) .* n0;
    d = sign(mu'*n0 - d) .* d;
    A(i,:) = n0';
    b(i) = d;
end
in_cvhull = all(A*pt >= b);


end

