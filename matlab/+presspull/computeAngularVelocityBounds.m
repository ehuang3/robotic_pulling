function [ l, u ] = computeAngularVelocityBounds( R, x0, y0, v_c, V, K )
%COMPUTEANGULARVELOCITYBOUNDS 
%   Compute angular velocity bounds for a given support region R, center of
%   pressure [x0; y0] and contact point velocity v_c.

%% Compute angular velocity bounds.
import presspull.*
% Parse inputs.
if nargin == 3
    v_c = 1;
end
assert(size(R,1) == 2, 'Dimension mismatch R');
assert(size(x0,1) == 1, 'Dimension mismatch x0');
assert(size(y0,1) == 1, 'Dimension mismatch y0');
assert(size(v_c,1) == 1, 'Dimension mismatch v_c');
assert(v_c > 0, 'Wrong direction');
% If center of pressure is aligned with the y-axis, the body translates.
if abs(x0) < 1e-5
    l = 0;
    u = 0;
    return
end
% Compute a feasible rotation center.
P_bd = generateCoMPressures(R,[x0;y0]);
K_bd = convhull(R');
R_bd = R(:,K_bd);
x_r = computeRotationCenter(R_bd,P_bd);
% 1.
l = 0;
u = -v_c/x_r;
w1 = bisectionSearch(R,x0,y0,v_c,l,u,V,K);
% 2.
l = -v_c/x_r;
u = -v_c/x_r;
while true
    l = 2*l;
    [G, Rx, Ry] = calcG2(R,-v_c/l,l,1,1,V,K);
    if ~pointInConvexHullG([x0,y0,0]', Rx, Ry, G)
        break;
    end
end
w2 = bisectionSearch(R,x0,y0,v_c,l,u,V,K);
% Return bounds.
l = min(w1,w2);
u = max(w1,w2);
end

function [ w ] = bisectionSearch( R, x0, y0, v_c, l, u, V, K )
    import presspull.*
    eps = 1e-7;
    while eps < abs(u-l)
        w = (u + l)/2;
        [G, Rx, Ry] = calcG2(R,-v_c/w,w,1,1,V,K);
        if pointInConvexHullG([x0,y0,0]', Rx, Ry, G)
            u = w;
        else
            l = w;
        end
    end
    w = (u + l)/2;
end