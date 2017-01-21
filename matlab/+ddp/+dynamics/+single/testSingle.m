%% Test double bound dynamics.
clc
clear
clear import

dynamics_namespace = 'double';

import presspull.*
import data.*
import(['ddp.dynamics.' dynamics_namespace '.autoF']);
import(['ddp.dynamics.' dynamics_namespace '.autoL']);
import(['ddp.dynamics.' dynamics_namespace '.autoLf']);
import(['ddp.dynamics.' dynamics_namespace '.autoH']);
import ddp.dft
import ddp.playback

%% Initialize dynamic system for rectangle object.
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
% Create rectangle.
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
% Fit bounds.
f = 1/(2*pi);
[ua,ub] = dft(rect.U,rect.T,f,100);
[la,lb] = dft(rect.L,rect.T,f,100);
%% Random start and end poses.
% Sample from left and right strips.
T0 = [(work_l/2)*rand + work_l/2;...
      (work_w-rect_w)*rand + rect_w/2;...
      pi];
T1 = [(work_l/2)*rand;...
      (work_w-rect_w)*rand + rect_w/2;...
      pi];
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
% Plot start and end poses.
if do_plot
    XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
    figure(1); clf; hold on; grid on;
    plot([0 0 work_l work_l 0],[0 work_w work_w 0 0],'k.-')
    XY = XYfunc(V0,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'b');
    plot(cp0(1),cp0(2),'b.');
    plot(cp0(1),cp0(2),'b.');
    XY = XYfunc(V1,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'r');
    plot(cp1(1),cp1(2),'r.');
    title('workspace'); xlabel('x'); ylabel('y')
    axis equal
    axis([0 work_l 0 work_w]);
end
%% DDP inputs.
cp = contact_point;
w0 = wrapTo2Pi(atan2(cp(2),cp(1)) + T0(3) + pi);
x0 = [cp0; w0; w0];
w1 = wrapTo2Pi(atan2(cp(2),cp(1)) + T1(3) + pi);
x1 = [cp1; w1; w1];

% Parameters.
N = 50;
p = [0.01 0.01 0.02 0.02];
k = 1000 * [5 5 1 1];
para = DDP_getDefaultPara;
para.maxIter = 500;
F = @(x,u,i) autoF(x,u,i,para,ua,ub,la,lb,f);
L = @(x,u,i) autoL(x,u,i,para);
Lf = @(x) autoLf(x,[],para,x1,p,k);
H = @(x,u,lambda,i) autoH(x,u,lambda,i,para,ua,ub,la,lb,f);

uLB = repmat([ 0.0 ;-2*pi],[1,N]);
uUB = repmat([ 0.02; 2*pi],[1,N]);

% Initialize controls.
v = (x1(1:2) - x0(1:2)) / N;
distance = norm(v) * N
t = wrapTo2Pi(atan2(v(2),v(1)));
u0 = [norm(v); t];
% u0 = [0; pi];
u_nom = repmat(u0,[1,N]);

%% DDP.
[xnom,unom,alpha_,beta_,info] = DDP(x0,N,u_nom,F,L,Lf,H,para,uLB,uUB);

%% Playback solution.
if do_plot
    figure(2);
    playback(rect,cp,xnom,unom,[]);
    XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
    hold on; grid on;
    plot([0 0 work_l work_l 0],[0 work_w work_w 0 0],'k.-')
    XY = XYfunc(V0,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'b');
    plot(cp0(1),cp0(2),'b.');
    plot(cp0(1),cp0(2),'b.');
    XY = XYfunc(V1,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'r');
    plot(cp1(1),cp1(2),'r.');
    title('workspace'); xlabel('x'); ylabel('y')
    axis equal
    axis([0 work_l 0 work_w]);
end
%% Plot controls.
if do_plot
    figure(3);
    subplot(311)
    cla; hold on; grid on;
    plot(xnom(3,:),'-*')
    plot(xnom(4,:),'-*')
    subplot(312)
    cla; hold on; grid on;
    plot(unom(2,:),'b-*')
    subplot(313)
    cla; hold on; grid on;
    plot(unom(1,:),'r-*')
end
