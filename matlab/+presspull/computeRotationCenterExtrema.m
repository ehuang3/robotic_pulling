function [ xl, xu ] = computeRotationCenterExtrema( R, x0, y0, V, k )
%COMPUTEROTATIONCENTEREXTREMA 
%   

%% 
import presspull.*
assert(size(R,1)==2,'Dimension mismatch');

% If CoP lies on the y-axis, the rotation center is at infinity.
if abs(x0) < 1e-15
    xl = inf;
    xu = inf;
    return;
end

% Reflect the object if the CoP falls in the left-half plane.
reflect = x0 < 0;
if reflect
    R(1,:) = -R(1,:);
    x0 = -x0;
end

% Compute a feasible rotation center.
[~, xr] = intersectLineLine([x0;y0],[y0;-x0],[0;0],[1;0]);

% Bisection search for minimum.
tol = 1e-7;
max_iters = 100;
i = 0;
l = 0;
u = xr;
err = u-l;
while tol < err && i < max_iters
    xr = (u+l)/2;
    [G_R, Rx, Ry] = calcG2(R,xr,-sign(xr),1,1,V,k);
    if pointInConvexHull([x0;y0;0],Rx,Ry,G_R)
        u = xr;
    else
        l = xr;
    end
    err = u-l;
    i = i+1;
end
xl = (u+l)/2;

% Compute a feasible rotation center.
[~, xr] = intersectLineLine([x0;y0],[y0;-x0],[0;0],[1;0]);

% Bisection search for maximum.
l = xr;
u = xr;
[G_R, Rx, Ry] = calcG2(R,u,-sign(u),1,1,V,k);
i = 0;
max_iters = 50;
while pointInConvexHull([x0;y0;0],Rx,Ry,G_R) && i < max_iters
    l = u;
    u = 2*u;
    [G_R, Rx, Ry] = calcG2(R,u,-sign(u),1,1,V,k);
    i = i+1;
end
if i == max_iters
    xu = inf;
    return;
end

err = u-l;
i = 0;
max_iters = 100;
while tol < err && i < max_iters
    xr = (u+l)/2;
    [G_R, Rx, Ry] = calcG2(R,xr,-sign(xr),1,1,V,k);
    if pointInConvexHull([x0;y0;0],Rx,Ry,G_R)
        l = xr;
    else
        u = xr;
    end
    err = u-l;
    i = i+1;
end
xu = u;

end

