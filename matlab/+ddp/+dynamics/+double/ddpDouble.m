function [ xnom, unom, valid, ferror ] = ddpDouble( rect, work, param, T0, T1, do_plot )
%DDPDOUBLE 
%   

%% Run DDP.
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

%% Object poses.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
V = rect.V; n_v = size(V,2);
com = rect.com;
cp = rect.cp;
V0 = rot(T0(3))*V + repmat(T0(1:2),[1,n_v]);
com0 = rot(T0(3))*com + T0(1:2);
cp0 = rot(T0(3))*cp + T0(1:2);
V1 = rot(T1(3))*V + repmat(T1(1:2),[1,n_v]);
com1 = rot(T1(3))*com + T1(1:2);
cp1 = rot(T1(3))*cp + T1(1:2);

% Plot start and end poses.
if do_plot
    XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
    figure(1);
    subplot(4,4,[1 2 5 6 9 10])
    cla; hold on; grid on;
    xn = work.xmin; xm = work.xmax; yn = work.ymin; ym = work.ymax;
    plot([xn xn xm xm xn],[yn ym ym yn yn],'k.-')
    XY = XYfunc(V0,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'b');
    plot(cp0(1),cp0(2),'b.');
    plot(cp0(1),cp0(2),'b.');
    XY = XYfunc(V1,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'r');
    plot(cp1(1),cp1(2),'r.');
    title('workspace'); xlabel('x'); ylabel('y')
    axis equal
    axis([xn xm yn ym]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DDP PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Slope of pseudo-Huber loss - [x, y, u, l, d] <- d is quadratic.
k = param.k;

% Convergence width of pseudo-Huber loss - [x, y, u, l]
p = param.p;

% Max DDP iterations.
max_iters = param.max_iters;

% Step-size for Dubin's paths.
dubin_step = param.dubin_step;

% Steering angle for computing Dubin's paths.
steering_angle = param.steering_angle;

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

% DDP inputs.
cp = rect.cp;
w0 = wrapTo2Pi(atan2(cp(2),cp(1)) + T0(3) + pi);
x0 = [cp0; w0; w0; 0];
w1 = wrapTo2Pi(atan2(cp(2),cp(1)) + T1(3) + pi);
x1 = [cp1; w1; w1; 0];

% Parameters.
N = size(u_nom,2);
para = DDP_getDefaultPara;
para.maxIter = max_iters;
f = 1/(2*pi);
ua = rect.ua;
ub = rect.ub;
la = rect.la;
lb = rect.lb;
F = @(x,u,i) autoF(x,u,i,para,ua,ub,la,lb,f);
L = @(x,u,i) autoL(x,u,i,para);
Lf = @(x) autoLf(x,[],para,x1,p,k);
H = @(x,u,lambda,i) autoH(x,u,lambda,i,para,ua,ub,la,lb,f);

uLB = repmat([ 0.0 ;-2*pi],[1,N]);
dmax = round(max(u_nom(1,:))+0.00499999,2);
uUB = repmat([ dmax; 2*pi],[1,N]);

% DDP.
[xnom,unom,alpha_,beta_,info] = DDP(x0,N,u_nom,F,L,Lf,H,para,uLB,uUB);
ferror = abs(x1 - xnom(:,end));

valid = and(all(and(work.xmin <= xnom(1,:),xnom(1,:) <= work.xmax)),all(and(work.ymin <= xnom(2,:),xnom(2,:) <= work.ymax)));


end

