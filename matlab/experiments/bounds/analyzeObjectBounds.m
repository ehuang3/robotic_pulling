%% Analyze bounds.
clc
clear

import presspull.*
import data.*

%% Timings. convergence.
load('/home/eric/src/presspull/data/bounds/mit_bounds.mat');

w = pi/2;
tol = 1/180*pi;
exact = zeros([2,length(bounds)]);
fifty = zeros([2,length(bounds)]);
peshkin = zeros([2,length(bounds)]);
for i = 1:length(bounds)
    obj = bounds(i)
    T = obj.exact.T;
    L = obj.exact.L;
    U = obj.exact.U;
    exact(1,i) = computeConvergenceTime(w,tol,T,L);
    exact(2,i) = computeConvergenceTime(w,tol,T,U);
    T = obj.fifty.T;
    L = obj.fifty.L;
    U = obj.fifty.U;
    fifty(1,i) = computeConvergenceTime(w,tol,T,L);
    fifty(2,i) = computeConvergenceTime(w,tol,T,U);
    T = obj.peshkin.T;
    L = obj.peshkin.L;
    U = obj.peshkin.U;
    peshkin(1,i) = computeConvergenceTime(w,tol,T,L);
    peshkin(2,i) = computeConvergenceTime(w,tol,T,U);
end

%%
disp('exact')
mu = mean(exact,2)
sigma = std(exact,0,2)

disp('fifty')
mu = mean(fifty,2)
sigma = std(fifty,0,2)

disp('peshkin')
mu = mean(peshkin,2)
sigma = std(peshkin,0,2)

