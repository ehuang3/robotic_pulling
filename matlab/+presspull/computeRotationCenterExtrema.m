function [ xl, xu ] = computeRotationCenterExtrema( R, x0, y0 )
%COMPUTEROTATIONCENTEREXTREMA 
%   

%% 
import presspull.*

% If CoP lies on the y-axis, the rotation center is at infinity.
if x0 == 0
    xl = inf;
    xu = inf;
    return;
end

% Reflect the object if the CoP falls in the left-half plane.
reflect = x0 < 0;
if reflect
    R(:,1) = -R(:,1);
    x0 = -x0;
end
R_x = R(:,1);
R_y = R(:,2);

% Compute a feasible rotation center.
[~, xr] = intersectLineLine([x0;y0],[y0;-x0],[0;0],[1;0]);

% Bisection search for minimum.
tol = 1e-7;
max_iters = 50;
i = 0;
l = 0;
u = xr;
err = u-l;
while tol < err && i < max_iters
    xr = (u+l)/2;
    G_R = G(R,xr,-sign(xr),1,1);
    if pointInConvexHull([x0;y0;0],R_x,R_y,G_R)
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
G_R = G(R,u,-sign(u),1,1);
i = 0;
max_iters = 50;
while pointInConvexHull([x0;y0;0],R_x,R_y,G_R) && i < max_iters
    l = u;
    u = 2*u;
    G_R = G(R,u,-sign(u),1,1);
    i = i+1;
end
if i == max_iters
    xu = inf;
    return;
end

err = u-l;
i = 0;
max_iters = 50;
while tol < err && i < max_iters
    xr = (u+l)/2;
    G_R = G(R,xr,-sign(xr),1,1);
    if pointInConvexHull([x0;y0;0],R_x,R_y,G_R)
        l = xr;
    else
        u = xr;
    end
    err = u-l;
    i = i+1;
end
xu = u;

end
