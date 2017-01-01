function [ b ] = computeAngularVelocityBounds( R, x0, v_c )
%COMPUTEANGULARVELOCITYBOUNDS 
%   Compute angular velocity bounds for a given support region R, center of
%   pressure x0 and contact point velocity v_c.

%% Compute angular velocity bounds.
import presspull.*
% Parse inputs.
if nargin == 3
    v_c = 1;
end
assert(size(R,1) == 2, 'Dimension mismatch R');
assert(size(x0,1) == 2, 'Dimension mismatch x0');
assert(size(v_c,1) == 1, 'Dimension mismatch v_c');
% If center of pressure is aligned with the y-axis, the body translates.
if x0 == 0
    b = [0, 0];
    return
end
% 1.
l = 0;
u = -v_c/(x0 + y0^2/x0);
w1 = bisectionSearch(R,x0,y0,v_c,l,u);
% 2.
l = -v_c/(x0 + y0^2/x0);
u = -v_c/(x0 + y0^2/x0);
while true
    u = 2*u;
    G_R = presspull.G(R,-v_c/u,u,1,1);
    if ~pointInConvexHull2([x0,y0,0]',[R; G_R])
        break;
    end
end
w2 = bisectionSearch(R,x0,y0,v_c,l,u);
% Return bounds.
b = [min(w1,w2), max(w1,w2)];
end

function [ w ] = bisectionSearch( R, x0, y0, v_c, l, u )
    eps = 1e-7;
    while eps < u-l
        w = (u + l)/2;
        G_R = presspull.G(R,-v_c/w,w,1,1);
        if pointInConvexHull2([x0,y0,0]',[R; G_R])
            u = w;
        else
            l = w;
        end
    end
    w = (u + l)/2;
end