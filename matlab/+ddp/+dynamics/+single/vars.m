%% 
% format compact

V.x = sym('x%d',[1,3],'real')
V.u = sym('u%d',[1,2],'real')
V.lambda = sym('l%d',[1,3],'real')
V.xf = sym('xf%d',[1,3],'real')
V.p = sym('p%d',[1,3],'real')
V.k = sym('k%d',[1,3],'real')
V.wa = sym('wa%d',[1,101],'real')
V.wb = sym('wb%d',[1,101],'real')
V.f = sym('f1','real')

save('/tmp/V.mat','-struct','V');
load('/tmp/V.mat');

tempF = {{'xnew','Fx','Fu'},{'autoF'},{'x','u','i','param','wa','wb','f'}};
tempL = {{'L'},{'autoL'},{'x','u','i','param'}};
tempLf = {{'Lf','Lfx','Lfxx'},{'autoLf'},{'x','i','param','xf','p','k'}};
tempH = {{'H','Hx','Hxx','Hu','Hux','Huu'},{'autoH'},{'x','u','lambda','i','param','wa','wb','f'}};

tempF{:}
tempL{:}
tempLf{:}
tempH{:}