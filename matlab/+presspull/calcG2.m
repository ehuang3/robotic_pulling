function [ G, Rx, Ry ] = calcG2( R, xr, w, mu, f0, V, k )
%CALCG2 Compute moment surface.
%   
%   Version 2 - Add indeterminant points (numerical stability).

%% 
import presspull.*

assert(size(R,1) == 2, 'Dimension mismatch');

% Get variables.
X = R(1,:);
Y = R(2,:);
dR = [];
dG = [];

% Check if rotation center is located within the object.
if ~isempty(V) && ~isempty(k)
    wn = 0;
    for i = 1:length(k)
        A = V(:,k(i,1));
        B = V(:,k(i,2));
        p = A;
        u = B-A;
        q = [xr;0];
        v = [1;0];
        [t,s] = intersectLineLine(p,u,q,v);
        if (s >= 0) && (0 <= t && t < 1)
            wn = wn + 1;
        end
    end
    if mod(wn,2) == 1
        % If in object, then add the indeterminant point G(x_r) to G and R.
        dR = [ xr xr; 0  0 ];
        dG = [-xr xr];
        % Also remove points epsilon close to x_r.
    end
end

% Compute moment surface G(R).
G = -mu.*f0.*sign(w).*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);

% Append indeterminant points.
R = [R dR];
Rx = R(1,:);
Ry = R(2,:);
G = [G dG];

end
