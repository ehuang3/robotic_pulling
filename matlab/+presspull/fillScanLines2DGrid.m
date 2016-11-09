function [ XX, YY, I ] = fillScanLines2DGrid( k, x, y, step )
%FILLSCANLINES2DGRID 
%   

%% 
import presspull.*

% step = 5e-2;

% Find min and max Y and X.
y_min = min(y);
y_max = max(y);
x_min = min(x);
x_max = max(x);

% Create grid.
X = x_min:step:x_max+step;
Y = y_min:step:y_max+step;
num_Y = length(Y);
[XX,YY] = meshgrid(X,Y);
I = zeros(size(XX));

% Fill grid.
for j = 1:num_Y
    % y-line.
    q = [0; Y(j)];
    v = [1;0];
    % Find line segment intersections.
    x_int = [];
    for i = 1:size(k,1)
        A = [x(k(i,1)), y(k(i,1))]';
        B = [x(k(i,2)), y(k(i,2))]';
        p = A;
        u = B-A;
        [t, ~] = intersectLineLine(p,u,q,v);
        if ~(0 <= t && t <= 1) || isinf(t)
            continue;
        end
        r = p + t*u;
        x_int(end+1) = r(1);
    end
    % Fill in scan line.
    x_int = sort(x_int);
    for i = 1:2:length(x_int)
        x0 = x_int(i);
        x1 = x_int(min(i+1,length(x_int)));
        % Nearest x index.
        i0 = round((x0 - x_min)/step)+1;
        i1 = round((x1 - x_min)/step)+1;
        I(i0:i1,j) = 1;
    end
end

end

