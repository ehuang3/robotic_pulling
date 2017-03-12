%% Plot angular velocity bounds.
clc
clear

import presspull.*
import data.*

%% Load tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];
R = fillScanLines2DGrid2(K,X',Y',1e-2);

% Plot.
figure(1)
cla; hold on; grid on; axis equal;
plot(X,Y,'k')
plot(R(1,:),R(2,:),'b.')

tetra.V = [X';Y'];
tetra.K = K;
tetra.com = CoP;
tetra.R = R;

%% Generate angular velocity bounds.
[ L, U, T ] = computeBounds( tetra, [0;0] )

%%

figure(1)
clf; axis normal; grid on; hold on;
fill([T, fliplr(T)], [L, fliplr(U)], 0.75 * ones([1, 3]))
% plot([T, fliplr(T)], [L, fliplr(U)],'k')
% plot(T,L,'k','LineWidth',2')
% plot(T,U,'k')
yl = ylabel('$\dot{\theta}$');
set(yl, 'Interpreter', 'latex')
xlabel \theta
a = axis;
grid on
hold on
fill([T, fliplr(T)]+2*pi, [L, fliplr(U)], 0.75 * ones([1, 3]))
fill([T, fliplr(T)]-2*pi, [L, fliplr(U)], 0.75 * ones([1, 3]))
blue = [0 113 188]/255;
orange = [216 82 24]/255;
plot(T,L,'color',blue,'linewidth',2)
plot(T,U,'color',orange,'linewidth',2)
plot(T+2*pi,L,'color',blue,'linewidth',2)
plot(T+2*pi,U,'color',orange,'linewidth',2)
plot(T-2*pi,L,'color',blue,'linewidth',2)
plot(T-2*pi,U,'color',orange,'linewidth',2)
% plot([T, fliplr(T)]+2*pi, [L, fliplr(U)], 'k')
% plot([T, fliplr(T)]-2*pi, [L, fliplr(U)], 'k')
axis(a)
ax = gca;
set(ax,'TickLabelInterpreter', 'latex');
ax.XTick = [-pi -pi/2 0 pi/2 pi];
ax.XTickLabel = {'$-\pi$', '$-\frac{\pi}{2}$', '0', '$\frac{\pi}{2}$', '$\pi$'};
ax.YTick = [-pi -pi/2 0 pi/2 pi];
ax.YTickLabel = {'$-\pi$', '$-\frac{\pi}{2}$', '0', '$\frac{\pi}{2}$', '$\pi$'};
% ax.YTick = [-pi 0 pi];
% ax.YTickLabel = {'$-\pi$', '0', '$\pi$'};
set(gcf, 'Color', 'w');
set(gca,'fontsize',14)
set(gca, 'Layer','top')
box on
set(gcf, 'Renderer', 'painters');
hp = findobj(gca,'type','patch');
hatchfill(hp)
ax.GridAlpha = 0.5
% export_fig src/presspull/tex/fig/omega_bounds1.eps -a1 -nocrop -native -m2 -a1 -eps

%% 

asldfkj