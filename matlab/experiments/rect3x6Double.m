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
import ddp.idft
import ddp.computeRearAxle
import ddp.planCSC

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SYSTEM PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Workspace bounds.
x_work_min = 0;
x_work_max = 0.91;
y_work_min = 0;
y_work_max = 0.61;

% A-B target locations. Robot will pull between A and B.
A = [0.30; 0.3];
B = [0.60; 0.3];
AB_side = 0.05;

% TODO: List of contact points relative to object frame.
% contact_point_list = [0.065; 0];

% Contact point to use.
contact_point = [0.065; 0];

% Percentage of surface always in contact with object.
percent_in_contact = 0.50;

%% Initialize dynamic system for rectangle object.
% 3x6 rectangle.
rect_w = 0.0762;
rect_l = 0.1524;
mass = 0.188;
% Robot.
g = 9.807;
zforce = g * 0.0;
% Plot.
do_plot = 1;
% Create rectangle.
rect = rectangle(rect_l/2, rect_w/2);
% Interpolate CoM.
total_mass = mass + zforce / g;
rect.com = (mass/total_mass)*rect.com + (zforce/g/total_mass)*contact_point;
% Add contact point.
rect.cp = contact_point;
% Compute bounds.
R = fillScanLines2DGrid2(rect.K,rect.V(1,:),rect.V(2,:),2.5e-3);
rect.R = R;
rect.LB = zeros([1,size(R,2)]);
rect.UB = 1 / (percent_in_contact * size(R,2)) * ones([1,size(R,2)]);
% rect.UB = ones([1,size(R,2)]);
[L,U,T] = computeBounds(rect, contact_point);
rect.L = L;
rect.U = U;
rect.T = T;
% Fit bounds.
f = 1/(2*pi);
[ua,ub] = dft(rect.U,rect.T,f,100);
[la,lb] = dft(rect.L,rect.T,f,100);
%% Plot bounds.
if do_plot
    figure(1);
    subplot(4,4,[13 14])
    cla; hold on; grid on;
    plot(rect.T,rect.U)
    plot(rect.T,rect.L)
    title('bounds')
    xlabel('\theta')
    ylabel('\theta''')
    axis tight
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% READ MOCAP AND ROBOT POSES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting pose.
T0 = [  2*AB_side*(rand-0.5) + A(1) ; ...
        2*AB_side*(rand-0.5) + A(2) ; ...
        2*rand*pi                   ];

% TODO: Angle needs to be in [0, 2pi).

%% Randomized end poses.
% Target pose.
dA = norm(T0(1:2)-A);
dB = norm(T0(1:2)-B);
C = B;
if dB < dA
    C = A;
end
T1 = [  2*AB_side*(rand-0.5) + C(1) ; ...
        2*AB_side*(rand-0.5) + C(2) ; ...
        2*rand*pi                   ];

%% Object poses.
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

%% Plot start and end poses.
if do_plot
    XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
    figure(1);
    subplot(4,4,[1 2 5 6 9 10])
    cla; hold on; grid on;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DDP PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Slope of pseudo-Huber loss - [x, y, u, l]
k = 1000 * [5 5 1 1];

% Convergence width of pseudo-Huber loss - [x, y, u, l]
p = [0.01 0.01 0.02 0.02];

% Max DDP iterations.
max_iters = 300;

% Step-size for Dubin's paths.
dubin_step = 1.5e-2;

% Steering angle for computing Dubin's paths.
steering_angle = 45/180 * pi;

%% CSC path.
[rac, rmin] = computeRearAxle(steering_angle,rect,rect.U);
s0 = rot(T0(3))*rac + T0(1:2);
s1 = rot(T1(3))*rac + T1(1:2);
h0 = cp0 - s0;
h1 = cp1 - s1;
[T,l] = planCSC(s0,h0,s1,h1,rmin,dubin_step);
dT = T(:,2:end) - T(:,1:end-1);
u_nom = [sqrt(dT(1,:).^2+dT(2,:).^2); atan2(dT(2,:),dT(1,:))];

if do_plot
    figure(1);
    subplot(4,4,[1 2 5 6 9 10])
    hold on; grid on;
    plot(s0(1),s0(2),'bo')
    plot(s1(1),s1(2),'ro')
    plot(T(1,:),T(2,:),'g.')
    plot(T(1,:),T(2,:),'g')
end

%% DDP inputs.
cp = rect.cp;
w0 = wrapTo2Pi(atan2(cp(2),cp(1)) + T0(3) + pi);
x0 = [cp0; w0; w0];
w1 = wrapTo2Pi(atan2(cp(2),cp(1)) + T1(3) + pi);
x1 = [cp1; w1; w1];

% Parameters.
N = size(u_nom,2);
para = DDP_getDefaultPara;
para.maxIter = max_iters;
F = @(x,u,i) autoF(x,u,i,para,ua,ub,la,lb,f);
L = @(x,u,i) autoL(x,u,i,para);
Lf = @(x) autoLf(x,[],para,x1,p,k);
H = @(x,u,lambda,i) autoH(x,u,lambda,i,para,ua,ub,la,lb,f);

uLB = repmat([ 0.0 ;-2*pi],[1,N]);
dmax = round(max(u_nom(1,:))+0.00499999,2);
uUB = repmat([ dmax; 2*pi],[1,N]);

%% Initialize controls.
% v = (x1(1:2) - x0(1:2)) / N;
% distance = norm(v) * N;
% t = wrapTo2Pi(atan2(v(2),v(1)));
% u0 = [norm(v); t];
% u0 = [0; 0];
% u_nom = repmat(u0,[1,N]);
% u_nom = [u_nom repmat(u0,[1,N-size(u_nom,2)])];

%% DDP.
[xnom,unom,alpha_,beta_,info] = DDP(x0,N,u_nom,F,L,Lf,H,para,uLB,uUB);

%% Playback solution.
if do_plot
    figure(1);
    subplot(4,4,[1 2 5 6 9 10]);
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
    figure(1);
    subplot(4,4,[3 4])
    cla; hold on; grid on; axis tight;
    plot(xnom(3,:),'-')
    plot(xnom(4,:),'-')
    title('u l')
    ylabel('radians');
    subplot(4,4,8)
    cla; hold on; grid on; axis tight;
    stem(unom(2,:),'b.')
    title('\phi'); ylabel('radians')
    subplot(4,4,7)
    cla; hold on; grid on; axis tight;
    stem(unom(1,:),'r.')
    title('d'); ylabel('m')
end
