function [ F_x ] = Fx( R, xr, w, mu )
%FX Summary of this function goes here
%   Detailed explanation goes here

%%
R_x = R(:,1);
R_y = R(:,2);
F_x = -mu.*sign(w).*(-R_y-xr)./sqrt((R_x-xr).^2 + (R_y).^2);

end

