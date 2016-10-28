%% Test plane convex hull intersection.

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];
plot(X,Y,'b');

% Rotate tetrapod.
theta = -pi/6;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
XY = rot * [X';Y'];
X_tetra = XY(1,:)';
Y_tetra = XY(2,:)';
CoP = rot * CoP;

% Compute intertior points.
pts = fillScanLines2D(K,X_tetra,Y_tetra);
X = pts(:,1);
Y = pts(:,2);

% Compute G(R).
f0 = 1;
u = 1;
xr = 0.75;
G = u.*f0.*(X.*X + Y.*Y - X.*xr)./sqrt((X-xr).^2 + Y.^2);
K = convhull(X,Y,G);

%% 
n = [0 0 1]';
d = 0;
XYZ = [X Y G];
pts = [];
for i = 1:size(K,1)
    p1 = XYZ(K(i,1),:)';
    p2 = XYZ(K(i,2),:)';
    p3 = XYZ(K(i,3),:)';
    P = XYZ(K(i,1:3),:)';
    for j = 1:3
        p = P(:,j);
        q = P(:,j+1-floor(j/3)*3);
        if abs(n'*p + n'*q) < 1e-7
            continue
        end
        t = (d - n'*q)./(n'*p-n'*q);
        if 0 <= t && t <= 1
            pts = [pts, t*p + (1-t)*q];
        end
    end
end
X_p = pts(1,:)';
Y_p = pts(2,:)';
K_p = convhull(X_p,Y_p);

hold on
XY_p = inv(rot) * [X_p';Y_p'];
X_p = XY_p(1,:)';
Y_p = XY_p(2,:)';
plot(X_p(K_p), Y_p(K_p),'r')
plot(0,0,'k*')
axis equal