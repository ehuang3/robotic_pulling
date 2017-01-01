%% Compute total frictional moment.
import presspull.*

% Initial values.
mu = 1;
f0 = 1;
r_IC = [0.5;0]';
w = -1;
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
R = fillScanLines2D(K,X,Y);
num_pts = size(R,1);
R_x = R(:,1);
R_y = R(:,2);
P = ones([num_pts,1]) ./ num_pts;

% Compute frictional moment.
xr = r_IC(1);
yr = r_IC(2);
G = -mu.*sign(w).*(R_x.^2+R_y.^2-R_x.*xr-R_y.*yr)./ ... 
     sqrt((R_x-xr).^2 + (R_y-yr).^2) .* P;
mf = sum(G);