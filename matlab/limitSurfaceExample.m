%% Create pressure distribution.
clear
clc

% Initial conditions.
m = 1.508; % kg
edge_len = [150 150 212.1] * 1e-3; % mm
height = 50 * 1e-3; % mm
sup_pt_loc = {[[10;10], [10;130], [130;10]] * 1e-3, ...
              [[30;30], [30;90], [90;30]] * 1e-3, ...
              [[10;10], [10;130], [90;30]] * 1e-3, ...
              [[30;30], [63.33;43.33], [43.33;63.33]] * 1e-3}; % mm
com = [edge_len(1)/3, edge_len(2)/3]';%, height/2]';

% Solve for forces.
sups = sup_pt_loc{1};
A = [1 1 1; 0 0 0; 0 0 0];
for i = 1:2
    u = sups(i,:) - com(i);
    A(i+1,:) = u;
end
b = [m * 9.81 0 0]';
x = A\b;
Fz = x;

%% Limit surface.
mu = 0.5; % Coulomb friction
N = 50;
num_sup = size(sups, 2);
F = zeros([3, N, num_sup]);
for i = 1:N
    for j = 1:num_sup
        % Compute friction force.
        theta = (i-1) / N * 2 * pi;
        Fx = cos(theta) * mu * Fz(j);
        Fy = sin(theta) * mu * Fz(j);
        % Compute moment.
        r = sups(:,j) - com;
        N0z = r(1) * Fy - r(2) * Fx;
        F(:,i,j) = [Fx Fy N0z];
    end
end

% Convex hull.
pts = zeros([N ^ num_sup, 3]);
done = false;
p = [1 1 1];
r = [N N N];
i = 1;
while ~done
    % Add forces.
    pts(i,:) = F(:,p(1),1) + F(:,p(2),2) + F(:,p(3),3);
    % Increment permutation and count.
    [p, done] = nextPermutation(p,r);
    i = i + 1;
end
K = convhull(pts(:,1), pts(:,2), pts(:,3));
trimesh(K, pts(:,1), pts(:,2), pts(:,3));
axis vis3d

%% Convexity of pressure.
% Initial conditions.
m = 1.508; % kg
edge_len = [150 150 212.1] * 1e-3; % mm
height = 50 * 1e-3; % mm
sup_pt_loc = {[[10;10], [10;130], [130;10]] * 1e-3, ...
              [[30;30], [30;90], [90;30]] * 1e-3, ...
              [[10;10], [10;130], [90;30]] * 1e-3, ...
              [[30;30], [63.33;43.33], [43.33;63.33]] * 1e-3}; % mm
com = [edge_len(1)/3, edge_len(2)/3]';%, height/2]';
com = [sym('cx', 'real') sym('cy', 'real')];

% Solve for forces.
sups = sup_pt_loc{1};
A = [1 1 1; 0 0 0; 0 0 0];
for i = 1:2
%     u = sups(i,:) - com(i);
    u = sups(i,:);
    A(i+1,:) = u;
end
b = [m * 9.81 0 0]';
x = A\b;
Fz = x;

u = [0; com'];
v = [1 1 1];
mg = [sym('mg'); 0; 0];