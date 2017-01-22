function [ l, u ] = computeAngularVelocityBounds( R, x0, y0, v_c, V, K, LB, UB )
%COMPUTEANGULARVELOCITYBOUNDS 
%   Compute angular velocity bounds for a given support region R, center of
%   pressure [x0; y0] and contact point velocity v_c.

%% Compute angular velocity bounds.
import presspull.*
% Parse inputs.
if nargin == 3
    v_c = 1;
end
if nargin < 7
    LB = zeros([1,size(R,2)]);
    UB = ones([1,size(R,2)]);
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
P = generateCoMPressures(R,[x0;y0],LB,UB);
% sum(P > 1e-6)
% K_bd = convhull(R');
% R_bd = R(:,K_bd);
x_r = computeRotationCenter(R,P);
% 1.
l = 0;
u = -v_c/x_r;
w1 = bisectionSearch(R,x0,y0,v_c,l,u,V,K,LB,UB);
% 2.
l = -v_c/x_r;
u = -v_c/x_r;
while true
    u = l;
    l = 2*l;
    [G, Rx, Ry, GLB, GUB] = calcG2(R,-v_c/l,l,1,1,V,K,LB,UB);
    if ~pointInConvexHullG2([x0,y0,0]', Rx, Ry, G, GLB, GUB)
        break;
    end
end
w2 = bisectionSearch(R,x0,y0,v_c,l,u,V,K,LB,UB);
% Return bounds.
l = min(w1,w2);
u = max(w1,w2);
end

function [ w ] = bisectionSearch( R, x0, y0, v_c, l, u, V, K, LB, UB )
    import presspull.*
    eps = 1e-7;
    while eps < abs(u-l)
        w = (u + l)/2;
        [G, Rx, Ry, GLB, GUB] = calcG2(R,-v_c/w,w,1,1,V,K,LB,UB);
        if pointInConvexHullG2([x0,y0,0]', Rx, Ry, G, GLB, GUB)
            u = w;
        else
            l = w;
        end
    end
    w = (u + l)/2;
end