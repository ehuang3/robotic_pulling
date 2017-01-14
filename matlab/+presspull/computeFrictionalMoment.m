function [ mf, d_mf ] = computeFrictionalMoment( mu, x_IC, w, R, P )
%COMPUTEFRICTIONALMOMENT 
%   

%% Compute the total frictional moment.
% Get X, Y of points in support region.
R_x = R(1,:);
R_y = R(2,:);

% Compute frictional moment.
xr = x_IC;
yr = 0;
G = -mu.*sign(w).*(R_x.^2+R_y.^2-R_x.*xr-R_y.*yr)./ ... 
     sqrt((R_x-xr).^2 + (R_y-yr).^2).*P;
G = G(~isnan(G));
mf = sum(G);
dG = -mu.*abs(xr).*(R_y.^2)./sqrt((R_x-xr).^2 + (R_y-yr).^2).^3.*P;
d_mf = sum(dG);

end

