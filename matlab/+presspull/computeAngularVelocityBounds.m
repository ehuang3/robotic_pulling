function [ b ] = computeAngularVelocityBounds( R, x0, y0, v_c )
%COMPUTEANGULARVELOCITYBOUNDS 
%   Compute angular velocity bounds for a given support region R and center
%   of pressure [x0, y0].

%% Compute angular velocity bounds.
if nargin == 3
    v_c = [0, 1]';
end
% Align coordinate frame with contact velocity.
if v_c(1) ~= 0
    
end
% If center of pressure is aligned with the y-axis, the body translates.
if x0 == 0
    b = [0, 0];
    return
end
% 1.
l = 0;
u = -norm(v_c)/(x0 + y0^2/x0);
w1 = bisectionSearch(R,x0,y0,v_c,l,u);
% 2.
l = -norm(v_c)/(x0 + y0^2/x0);
u = -norm(v_c)/(x0 + y0^2/x0);
while true
    u = 2*u;
    G_R = presspull.G(R,-norm(v_c)/u,u,1,1);
    if ~pointInConvHull([x0,y0,0]',R_x,R_y,G_R)
        break;
    end
end
w2 = bisectionSearch(R,x0,y0,v_c,l,u);
% Return bounds.
b = [min(w1,w2), max(w1,w2)];
end

function [ w ] = bisectionSearch(R, x0, y0, v_c, l, u)
    
end