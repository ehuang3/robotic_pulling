%% Analyze bounds.
clc
clear

import presspull.*
import data.*

%% timeins
load('/home/eric/src/presspull/data/bounds/3pod_bounds.mat');

w = pi/2;
tol = 1/180*pi;
    e = zeros([2,length(bounds)]);
    f = zeros([2,length(bounds)]);
    p = zeros([2,length(bounds)]);
for i = 1:length(bounds)
    obj = bounds(i);
    T = obj.exact.T;
    L = obj.exact.L;
    U = obj.exact.U;
    U(150) - L(150);
    e(1,i) = computeConvergenceTime(w,tol,T,L);
    e(2,i) = computeConvergenceTime(w,tol,T,U);
    T = obj.fifty.T;
    L = obj.fifty.L;
    U = obj.fifty.U;
    U(150) - L(150);
    f(1,i) = computeConvergenceTime(w,tol,T,L);
    f(2,i) = computeConvergenceTime(w,tol,T,U);
    T = obj.peshkin.T;
    L = obj.peshkin.L;
    U = obj.peshkin.U;
    U(150) - L(150);
    p(1,i) = computeConvergenceTime(w,tol,T,L);
    p(2,i) = computeConvergenceTime(w,tol,T,U);
end

%
disp('exact')
mu = mean(e,2)
sigma = std(e,0,2)

disp('fifty')
mu = mean(f,2)
sigma = std(f,0,2)

disp('peshkin')
mu = mean(p,2)
sigma = std(p,0,2)

%% Timings. convergence.
load('/home/eric/src/presspull/data/bounds/2pod_bounds.mat');

w = linspace(1/pi,5/pi,25)
tol = 1/180*pi;
exact = struct;
fifty = struct;
peshkin = struct;
for j = 1:length(w)
    e = zeros([2,length(bounds)]);
    f = zeros([2,length(bounds)]);
    p = zeros([2,length(bounds)]);
    for i = 1:length(bounds)
        obj = bounds(i);
        T = obj.exact.T;
        L = obj.exact.L;
        U = obj.exact.U;
        U(150) - L(150);
        e(1,i) = computeConvergenceTime(w(j),tol,T,L);
        e(2,i) = computeConvergenceTime(w(j),tol,T,U);
        T = obj.fifty.T;
        L = obj.fifty.L;
        U = obj.fifty.U;
        U(150) - L(150);
        f(1,i) = computeConvergenceTime(w(j),tol,T,L);
        f(2,i) = computeConvergenceTime(w(j),tol,T,U);
        T = obj.peshkin.T;
        L = obj.peshkin.L;
        U = obj.peshkin.U;
        U(150) - L(150);
        p(1,i) = computeConvergenceTime(w(j),tol,T,L);
        p(2,i) = computeConvergenceTime(w(j),tol,T,U);
    end
    exact{j} = e;
    fifty{j} = f;
    peshkin{j} = p;
end

%% boxplot




for i = 1:length(w)
    EL = [EL exact{i}(1,:)'];
    EU = [EU exact{i}(2,:)'];
end

figure(1); clf; hold on; grid on; 

%%
disp('exact')
mu = mean(e,2)
sigma = std(e,0,2)

disp('fifty')
mu = mean(f,2)
sigma = std(f,0,2)

disp('peshkin')
mu = mean(p,2)
sigma = std(p,0,2)

