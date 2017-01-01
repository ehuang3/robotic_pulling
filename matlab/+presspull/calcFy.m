function [ F_y ] = Fy( R, xr, w, mu )
%FY Summary of this function goes here
%   Detailed explanation goes here

%%
R_x = R(:,1);
R_y = R(:,2);
F_y = -mu.*sign(w).*(R_x)./sqrt((R_x-xr).^2 + (R_y).^2);

end

