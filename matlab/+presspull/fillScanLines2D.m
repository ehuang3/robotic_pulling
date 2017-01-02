function [ pts ] = fillScanLines2D( k, x, y )
%FILLSCANLINES2D
%   

%%
import presspull.*

step = 1e-2;

% Find min and max Y.
y_min = min(y);
y_max = max(y);

pts = [];
n_y = ceil((y_max - y_min)/step)+1;
for dy = linspace(0, y_max-y_min, n_y)
    % y-line.
    q = [0; y_min+dy];
    v = [1;0];
    % Find line segment intersections.
    x_int = [];
    for i = 1:size(k,1)
        A = [x(k(i,1)), y(k(i,1))]';
        B = [x(k(i,2)), y(k(i,2))]';
        p = A;
        u = B-A;
        [t, s] = intersectLineLine(p,u,q,v);
        if ~(0 <= t && t <= 1) || isinf(t)
            continue;
        end
        r = p + t*u;
        x_int(end+1) = r(1);
    end
    % Fill in scan line.
    x_int = sort(x_int);
    sl = [];
    for i = 1:2:length(x_int)
        x_min = x_int(i);
        x_max = x_int(min(i+1,length(x_int)));
        n_x = ceil((x_max - x_min)/step)+1;
        sl = [sl; linspace(x_min, x_max, n_x)'];
    end
    pts = [pts; [sl (y_min+dy).*ones([size(sl,1),1])]];
end
% Convert to row vector -- one point per column.
pts = pts';

end

