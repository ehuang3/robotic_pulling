%% Experimental setup for rect3x6.
clc
clear

import presspull.*
import ddp.*
import data.*

rosinit

%% Parameters.

% 3x6 rectangle.
rect_w = 0.0762;
rect_l = 0.1524;
mass = 0.188;

% Robot.
g = 9.807;
zforce = g * 0.100;
contact_point = [0.065; 0];

% Workspace.
work_w = 0.61;
work_l = 0.91;

% Plot.
do_plot = 1;

%% Create rectangle.
% Init.
rect = rectangle(rect_l/2, rect_w/2);

% Interpolate CoM.
total_mass = mass + zforce / g;
rect.com = (mass/total_mass)*rect.com + (zforce/g/total_mass)*contact_point;

% Compute bounds.
rect.R = fillScanLines2DGrid2(rect.K,rect.V(1,:),rect.V(2,:),3e-3);
[L,U,T] = computeBounds(rect, contact_point);
rect.L = L;
rect.U = U;
rect.T = T;

%% Plot.
blue = [0 113 188]/255; orange = [216 82 24]/255;
if do_plot
    figure(1);
    clf;
    subplot(211)
    hold on; grid on; axis equal;
    V = rect.V; K = rect.K;
    X = [V(1,K(:,1)); V(1,K(:,2))];
    Y = [V(2,K(:,1)); V(2,K(:,2))];
    plot(X,Y,'k')
    plot(rect.com(1),rect.com(2),'k*')
    plot(contact_point(1),contact_point(2),'r.')
%     plot(rect.R(1,:),rect.R(2,:),'b.')
    title('object'); xlabel('x'); ylabel('y');
    subplot(212)
    hold on; grid on;
    plot(rect.T, rect.L,'color',blue);
    plot(rect.T, rect.U,'color',orange);
    title('phase'); xlabel('\theta'); ylabel('\theta''');
end

%% Start and end poses.
% Random poses.
T0 = [(work_l-rect_l)*rand + rect_l/2;...
      (work_w-rect_w)*rand + rect_w/2;...
      2*pi*rand];
T1 = [(work_l-rect_l)*rand + rect_l/2;...
      (work_w-rect_w)*rand + rect_w/2;...
      2*pi*rand];

% Object poses.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
V = rect.V; n_v = size(V,2);
com = rect.com;
cp = contact_point;
V0 = rot(T0(3))*V + repmat(T0(1:2),[1,n_v]);
com0 = rot(T0(3))*com + T0(1:2);
cp0 = rot(T0(3))*cp + T0(1:2);
V1 = rot(T1(3))*V + repmat(T1(1:2),[1,n_v]);
com1 = rot(T1(3))*com + T1(1:2);
cp1 = rot(T1(3))*cp + T1(1:2);

% Plot.
if do_plot
    figure(2); clf; hold on; grid on;
    plot([0 0 work_l work_l 0],[0 work_w work_w 0 0],'k.-')
    X0 = [V0(1,K(:,1)); V0(1,K(:,2))]; 
    Y0 = [V0(2,K(:,1)); V0(2,K(:,2))];
    plot(X0,Y0,'b');
    plot(cp0(1),cp0(2),'b*');
    X1 = [V1(1,K(:,1)); V1(1,K(:,2))];
    Y1 = [V1(2,K(:,1)); V1(2,K(:,2))];
    plot(X1,Y1,'r');
    plot(cp1(1),cp1(2),'r*');
    title('workspace'); xlabel('x'); ylabel('y')
    axis equal
    axis([0 work_l 0 work_w]);
end

%% DDP parameters.
% X0, X1 - start end.
cp = contact_point;
w0 = atan2(cp(2),cp(1)) + T0(3) + pi;
X0 = [cp0; w0; w0];
w1 = atan2(cp(2),cp(1)) + T1(3) + pi;
X1 = [cp1; w1; w1];

% Fit bounds.
f = 1/(2*pi);
[ua,ub] = dft(rect.U,rect.T,f,100);
[la,lb] = dft(rect.L,rect.T,f,100);

%% Parameters.
total_time = 5;
N = 100;
dt = total_time/N;
del = [0.001 0.001 0.01 0.01];%5 * [1 1 1 1];
kel = 10000 * [5 5 1 1];
para = DDP_getDefaultPara;
para.maxIter = 500;
F = @(x,u,i) autoF(x,u,i,para,ua,ub,la,lb,f,dt);
L = @(x,u,i) autoL(x,u,i,para,dt);
Lf = @(x) autoLf(x,i,para,X1,del,kel);
H = @(x,u,lambda,i,param) autoH(x,u,lambda,i,para,ua,ub,la,lb,f,dt);

% Initialize controls.
v0 = (X0(1:2) - X1(1:2))/total_time;
u0 = [norm(v0); atan2(v0(2),v0(1))];
u_nom = repmat(u0, [1 N]);
% u_nom = zeros([2,N]);

uLB = repmat([-0.1; -2*pi],[1,N]);
uUB = repmat([0.1; 2*pi],[1,N]);

%% DDP.
[xnom,unom,alpha_,beta_,info] = DDP(X0,N,u_nom,F,L,Lf,H,para,uLB,uUB);

%% Plot.
if do_plot
    playback(rect,cp,xnom,unom,[]);
end


