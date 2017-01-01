function [ ff ] = computeFrictionalForce( mu, xr, w, R, P )
%COMPUTEFRICTIONALFORCE
%   

%% Compute the total frictional force.
import presspull.*

% % Get X, Y of points in support region.
% R_x = R(:,1);
% R_y = R(:,2);

% Compute frictional force.
% F_x = -mu.*sign(w).*(-R_y-xr)./sqrt((R_x-xr).^2 + (R_y-0).^2).*P;
% F_y = -mu.*sign(w).*(R_x-0)./sqrt((R_x-xr).^2 + (R_y-0).^2).*P;
F_x = calcFx(R,xr,w,mu).*P;
F_y = calcFy(R,xr,w,mu).*P;
ff = [sum(F_x); sum(F_y)];

end

