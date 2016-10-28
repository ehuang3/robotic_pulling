function [ xr, mf ] = computeRotationCenter( R, P )
%COMPUTEROTATIONCENTER 
%   

%% 
import presspull.*

% Compute the center of pressure.
CoP = R'*P;

% If the CoP lies in the negative x plane, flip it.
if CoP(1) < 0
    R(:,1) = -R(:,1);
end
w = -1;
mu = 1;

% Bisection method.
xl = 0;
xu = 1;
tol = 1e-9;
l = computeFrictionalMoment(mu,xl,w,R,P);
u = computeFrictionalMoment(mu,xu,w,R,P);
mf = inf;
max_iters = 30;
i = 0;
while abs(mf) > tol && i < max_iters
    d = floor(abs(sign(l)-sign(u))/2);
    p = -(abs(sign(l)-sign(u))-1);
    xr = xu + (-1)^d*2^p*(xu-xl);
    mf = computeFrictionalMoment(mu,xr,w,R,P);
    if d == 0
        xl = xu;
        xu = xr;
        l = u;
        u = mf;
    elseif sign(l) == sign(mf)
        xl = xr;
        l = mf;
    else
        xu = xr;
        u = mf;
    end
    i = i + 1;
end

% If the CoP lies in the negative x plane, unflip the rotation center.
if CoP(1) < 0
    xr = -xr;
end

end

