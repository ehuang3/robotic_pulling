function [ rac, rmin ] = computeRearAxel( w, object, Omega )
%COMPUTEREARAXEL 
%   

%% Compute the center of the rear axel for w turning.

T = object.T;
L = object.L;
U = object.U;
if nargin < 3
    Omega = U;
end
% subplot(211)
% cla; hold on; grid on;
% plot(T,L)
% plot(T,U)

V = object.V;
K = object.K;
com = object.com;
cp = object.cp;
% subplot(212)
% cla; hold on; axis equal; grid on;
% plot(V(1,:),V(2,:),'k')
% plot(cp(1),cp(2),'r*')
% plot(com(1),com(2),'k*')
dp = 0.05 * [cos(-w); sin(-w)];
line = @(p,q) [p q];
pull = line(cp,cp+dp);
% plot(pull(1,:),pull(2,:),'g')
xr = -1/linterp(T,Omega,w);
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
xa = rot(-pi/2) * dp / norm(dp);
xr = xr * xa;
turn = line(cp,xr+cp);
% plot(turn(1,:),turn(2,:),'r')

nc = (cp-com)/norm(cp-com);
ic = xr + cp;
rac = com + (ic'*nc)*nc;
rmin = norm(ic-rac);
% plot(rac(1),rac(2),'r.')

end

