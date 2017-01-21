%% Test point robot dynamics.
clc
clear
clear import

import data.*
import presspull.*
import ddp.dynamics.pointR.autoF
import ddp.dynamics.pointR.autoL
import ddp.dynamics.pointR.autoLf
import ddp.dynamics.pointR.autoH

% Init.
x0 = [1;  1];
xN = [4; -3];

% Parameters.
N = 100;
p = [0.01 0.01];
k = 100 * [1 1];
para = DDP_getDefaultPara;
F = @(x,u,i) autoF(x,u,i,para);
L = @(x,u,i) autoL(x,u,i,para);
Lf = @(x) autoLf(x,[],para,xN,p,k);
H = @(x,u,lambda,i,param) autoH(x,u,lambda,i,para);

uLB = repmat([ 0  ;    0],[1,N]);
uUB = repmat([ 0.5; 2*pi],[1,N]);

% Initialize controls.
v = (xN - x0) / N
t = wrapTo2Pi(atan2(v(2),v(1)))
u0 = [0; t];
u_nom = repmat(u0,[1,N]);

%% DDP.
[xnom,unom,alpha_,beta_,info] = DDP(x0,N,u_nom,F,L,Lf,H,para,uLB,uUB);

%% Plot.
figure(1)
subplot(211)
cla; hold on; grid on; axis equal;
plot(xnom(1,:),xnom(2,:),'b.')
plot(x0(1),x0(2),'k*')
plot(xN(1),xN(2),'r*')
subplot(212)
cla; hold on; grid on;
plot(unom(2,:))


