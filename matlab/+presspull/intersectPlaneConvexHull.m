function [ X_p, Y_p, K_p ] = intersectPlaneConvexHull( n, d, X, Y, Z )
%INTERSECTPLANECONVEXHULL 
%   

%%
% n = [0 0 1]';
% d = 0;
K = convhull(X,Y,Z);
XYZ = [X Y Z];
pts = [];
for i = 1:size(K,1)
    P = XYZ(K(i,1:3),:)';
    for j = 1:3
        p = P(:,j);
        q = P(:,j+1-floor(j/3)*3);
        if abs(n'*p + n'*q) < 1e-7
            continue
        end
        t = (d - n'*q)./(n'*p-n'*q);
        if 0 <= t && t <= 1
            pts = [pts, t*p + (1-t)*q];
        end
    end
end
X_p = pts(1,:)';
Y_p = pts(2,:)';
K_p = convhull(X_p,Y_p);

end

