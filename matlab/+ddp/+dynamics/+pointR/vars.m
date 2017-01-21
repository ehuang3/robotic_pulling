%% 
% format compact

V.x = sym('x%d',[1,2],'real')
V.u = sym('u%d',[1,2],'real')
V.lambda = sym('l%d',[1,2],'real')
V.xf = sym('xf%d',[1,2],'real')
V.p = sym('p%d',[1,2],'real')
V.k = sym('k%d',[1,2],'real')

save('/tmp/V.mat','-struct','V');
load('/tmp/V.mat');

tempF = {{'xnew','Fx','Fu'},{'autoF'},{'x','u','i','param'}};
tempL = {{'L'},{'autoL'},{'x','u','i','param'}};
tempLf = {{'Lf','Lfx','Lfxx'},{'autoLf'},{'x','i','param','xf','p','k'}};
tempH = {{'H','Hx','Hxx','Hu','Hux','Huu'},{'autoH'},{'x','u','lambda','i','param'}};

tempF{:}
tempL{:}
tempLf{:}
tempH{:}