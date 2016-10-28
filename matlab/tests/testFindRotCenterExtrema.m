%% Find the extrema of rotation centers.

X = linspace(0,1);
Y = linspace(-0.5,0.5);
[X,Y] = meshgrid(X,Y);

x0 = 0.5;
y0 = 0.25;
f0 = 1;
u = 1;

xr = 1.5;

G = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);
surf(X,Y,G);

X = X(:);
Y = Y(:);
G = G(:);
K = convhull(X,Y,G);
trimesh(K,X,Y,G)

inConvexHull([x0;y0;0],X,Y,G)