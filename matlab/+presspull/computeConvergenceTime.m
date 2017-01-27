function [ t ] = computeConvergenceTime( w, tol, T, Omega )
%COMPUTECONVERGENCETIME 
%   

%% 
import presspull.*

odeFun = @(s,y) linterp(T,Omega,wrapToPi(y));
eventFun = @(s,y) convergeEvent(s,y,sign(w)*tol,[]);
opts=odeset('Events',eventFun);
[s,~] = ode45(odeFun,[0,inf],w,opts);
% ode45(odeFun,[0,inf],w,opts) % plot
t = max(s);

end

