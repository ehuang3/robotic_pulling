function [ G_R ] = calcG( R, xr, w, mu, f0 )
%G
%

%% 
assert(size(R,1) == 2, 'Dimension mismatch');

X = R(1,:);
Y = R(2,:);
G_R = -mu.*f0.*sign(w).*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);

end

