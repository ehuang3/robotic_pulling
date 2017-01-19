%% Test DDP.
clc
clear

import data.*
import ddp.*
import presspull.*

%% Compute angular velocity bounds.
% Load tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.40;
CoP = [0.40; 0];
R = fillScanLines2DGrid2(K,X',Y',1e-2);
tetra.V = [X';Y'];
tetra.K = K;
tetra.com = CoP;
tetra.R = R;

% Generate angular velocity bounds.
[L, U, T] = computeBounds(tetra, [0;0]);

%% Fit Fourier series to bounds.
f = 1/(2*pi);
w = linspace(-pi,pi,100);
[la,lb] = dft(L,T,f,10);
sl = idft(w,la,lb,f);
[ua,ub] = dft(U,T,f,10);
su = idft(w,ua,ub,f);

%% Test discretized dynamics.
total_time = 2;
n_step = 100;
dt = total_time / n_step;

% Random starting location.
x0 = [5 * rand([2,1]); 2*pi*rand*ones([2,1])-pi]

% Random ending location.
xN = [5 * rand([2,1]); 2*pi*rand*ones([2,1])-pi]

%% Get controls.
u = zeros([2,1]);
v = (xN(1:2) - x0(1:2))/total_time;
u(1) = norm(v);
u(2) = atan2(v(2),v(1));

% Simulate dynamics.
X = zeros([4,n_step]);
U = zeros([2,n_step]);
X(:,1) = x0;
U(:,1) = u;
for i = 2:n_step
    xp = X(:,i-1);
%     [~, xn] = symF(xp,u,ua,ub,la,lb,f,dt);
    xn = calcF(xp,u,ua,ub,la,lb,f,dt);
    X(:,i) = xn;
    U(:,i) = u;
end

% Animate dynamics.


%% Test loss function.


%% Test final loss function.


%% Test pseudo-Hamiltonian.

