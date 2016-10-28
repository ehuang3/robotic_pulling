function [ ff ] = computeFrictionalForce( mu, r_IC, w, R, P )
%COMPUTEFRICTIONALFORCE
%   

%% Compute the total frictional force.
% Get X, Y of points in support region.
R_x = R(:,1);
R_y = R(:,2);

% Compute frictional force.
xr = r_IC(1);
yr = r_IC(2);
F_x = -mu.*sign(w).*(-R_y-xr)./sqrt((R_x-xr).^2 + (R_y-yr).^2).*P;
F_y = -mu.*sign(w).*(R_x-yr)./sqrt((R_x-xr).^2 + (R_y-yr).^2).*P;
ff = [sum(F_x); sum(F_y)];

end

