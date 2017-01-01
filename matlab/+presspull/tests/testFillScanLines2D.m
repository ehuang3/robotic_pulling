%% Scan line fill in 2D.

% 2D Convex hull.
N = 10;
x = rand([10,1]);
y = rand([10,1]);
k = convhull(x,y);
hold on
plot(x,y,'b+');
plot(x(k),y(k),'r');

k = [k(1:end-1), k(2:end)];

%% Scan line.
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
    for i = 1:2:size(x_int,1)
        x_min = x_int(i);
        x_max = x_int(i+1);
        n_x = ceil((x_max - x_min)/step)+1;
        sl = [sl; linspace(x_min, x_max, n_x)'];
    end
    pts = [pts; [sl (y_min+dy).*ones([size(sl,1),1])]];
end

plot(pts(:,1), pts(:,2),'.')

%% Test 2D tetrapod.
import presspull.*

[X, Y, K] = generate2DTetrapod(0.5,0.7,0);

hold on
plot(X,Y,'r')

pts = fillScanLines2D(K,X,Y);
plot(pts(:,1),pts(:,2),'b.')
axis equal
