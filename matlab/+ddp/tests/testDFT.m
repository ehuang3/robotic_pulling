%% Test discrete fourier transform.
clc
clear

import presspull.*
import ddp.*
import data.*

%% Compute angular velocity bounds.
% Load tetrapod.
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

% Generate angular velocity bounds.
[ L, U, T ] = computeBounds( tetra, [0;0] )

%% Plot bounds.
figure (1)
subplot(211)
cla; hold on; axis auto; grid on;
plot(T,L,'k-')
plot(T,U,'k-')

% Fit Fourier series to bounds.
f = 1/(2*pi);

w = linspace(-pi,pi,1);
w = T;
[la,lb] = dft(L,T,f,10);
sl = idft(w,la,lb,f);
[ua,ub] = dft(U,T,f,10);
su = idft(w,ua,ub,f);

plot(w,sl)
plot(w,su)

subplot(212)
cla; hold on; axis auto; grid on;
plot(w,sl-L)
plot(w,su-U)

%% 




