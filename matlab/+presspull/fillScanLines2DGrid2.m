function [ pts ] = fillScanLines2DGrid2( k, x, y, step )
%FILLSCANLINES2DGRID2 
%   

%% 
import presspull.*

assert(size(k,2)==2,'Dimension mismatch');
% assert(size(k,1)==length(x)-1,'Dimension mismatch');
% assert(size(k,1)==length(y)-1,'Dimension mismatch');

if nargin == 3
    step = 1e-2;
end

%% Find min and max X,Y.
x_min = min(x);
x_max = max(x);
y_min = min(y);
y_max = max(y);

%% Adjust min/max onto grid of 'step' size.
x_min = floor(x_min/step)*step;
x_max = ceil(x_max/step)*step;
y_min = floor(y_min/step)*step;
y_max = ceil(y_max/step)*step;

%% Search for grid points within the input polygon.
x_num = round((x_max - x_min)/step);
y_num = round((y_max - y_min)/step);
x_pts = x_min + (0:x_num)*step;
y_pts = y_min + (0:y_num)*step;

pts = [];
for xp = x_pts
    for yp = y_pts
        % Compute winding number.
        w = 0;
        for i = 1:size(k,1)
            A = [x(k(i,1)), y(k(i,1))]';
            B = [x(k(i,2)), y(k(i,2))]';
            p = A;
            u = B-A;
            q = [xp;yp];
            v = [0;1];
            [t,s] = intersectLineLine(p,u,q,v);
            if (s >= 0) && (0 <= t && t < 1)
                w = w + 1;
            end
        end
        if mod(w,2) == 1
            pts = [pts q];
        end
    end
end

%% Add polygon boundary as additional points.
d = 0;
for i = 1:size(k,1)
                A = [x(k(i,1)), y(k(i,1))]';
            B = [x(k(i,2)), y(k(i,2))]';
    d = d + norm(B-A);
end
n_cp = ceil(d/step);
bd = getSpacedContactPoints([x;y],k,n_cp);
pts = [pts bd];

end

