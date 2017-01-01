function [ C ] = computeCoMPolygon( V )
%COMPUTECOM 
%   V - vertices
%   C - center of mass

%% Compute center of mass (centroid) of polygon.
assert(size(V,1)==2);
assert(norm(V(:,1) - V(:,end)) < 1e-7);

A = polyarea(V(1,:),V(2,:));
X = V(1,:);
Y = V(2,:);
Cx = 1./(6*A).* sum( (X(1:end-1)+X(2:end)).*(X(1:end-1).*Y(2:end)-X(2:end).*Y(1:end-1)) );
Cy = 1./(6*A).* sum( (Y(1:end-1)+Y(2:end)).*(X(1:end-1).*Y(2:end)-X(2:end).*Y(1:end-1)) );
C = [Cx; Cy];

end

