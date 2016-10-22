%% Generate COM trajectory.

edges = [150 150 212.1] * 1e-3; % mm
height = 50 * 1e-3; % mm
sup_pt_loc = {[[10;10], [10;130], [130;10]] * 1e-3, ...
              [[30;30], [30;90], [90;30]] * 1e-3, ...
              [[10;10], [10;130], [90;30]] * 1e-3, ...
              [[30;30], [63.33;43.33], [43.33;63.33]] * 1e-3}; % mm
com = [edges(1)/3, edges(2)/3]';%, height/2]';

sups = sup_pt_loc{1};

N = 50;
rad = 0.025;
com_traj = zeros([2, N]);
for i = 1:N
    theta = (i-1) / N * 2 * pi;
    dx = cos(theta) * rad;
    dy = sin(theta) * rad;
    com_traj(:,i) = [com(1) + dx; com(2) + dy];
end

for i = 1:N
    % Compute limit surface.
    com_t = com_traj(:,i);
    [K, X, Y, Z] = computeLimitSurfaceCOM(com_t, sups);
    % Plot.
    subplot(121);
    trimesh(K,X,Y,Z);
%     axis vis3d
    xlim([-12.5, 12.5])
    ylim([-12.5, 12.5])
    zlim([-1, 1])
%     xlim manual
%     ylim manual
%     zlim manual
    subplot(122);
    plotTriangle(edges, com_t, sups);
    pause(0.25);
end