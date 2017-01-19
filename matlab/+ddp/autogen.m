%% Autogenerate dynamics functions for DDP.

n_dft = 100;

f = sym('f','real');
dt = sym('dt','real');
ua = [sym('ua0','real') sym('ua%d',[1 n_dft],'real')];
ub = [sym('ub0','real') sym('ub%d',[1 n_dft],'real')];
la = [sym('la0','real') sym('la%d',[1 n_dft],'real')];
lb = [sym('lb0','real') sym('lb%d',[1 n_dft],'real')];

I = symIDFT([], ua, ub, f);

sym2exp(I);

[F, Fx, Fu] = symF([],[],ua,ub,la,lb,f,dt);
L = symL([],[],dt);
[Lf, Lfx, Lfxx] = symLf([],[],[]);
[H,Hx,Hxx,Hu,Hux,Huu] = symH([],[],ua,ub,la,lb,f,dt);

% f 	- a function describing system dynamics:
% 			[xnew,fx,fu] = f(x,u,i,para)
% L 	- a function descrbing running cost:
%			[L] = L(x,u,i,para)
% F 	- a function calculating the final cost:
% 			[F,Fx,Fxx] = F(x,i,para)
% H 	- a function calculating the pseudo-Hamiltonian:
% 			H(x,u,lambda) = L(x,u) + lambda^T*f(x,u)
% 			[H,Hx,Hxx,Hu,Hux,Huu] = H(x,u,lambda,i,para)

%% Get autogen path.
import data.*
root_path = fileparts(getDataPath);
autogen_path = fullfile(root_path,'matlab','+ddp');

%% Autogenerate dynamics.
F_path = fullfile(autogen_path,'autoF.m');

E = ['function [ x2, fx, fu ] = autoF( x1, u1, i, para, ua, ub, la, lb, f, dt )' char(10)];
E = [E 'x = x1(1);' char(10)];
E = [E 'y = x1(2);' char(10)];
E = [E 'u = x1(3);' char(10)];
E = [E 'l = x1(4);' char(10)];
E = [E 'v = u1(1);' char(10)];
E = [E 'p = u1(2);' char(10)];
for i = 0:n_dft
    E = [E sprintf('ua%d = ua(%d);',i,i+1) char(10)];
    E = [E sprintf('ub%d = ub(%d);',i,i+1) char(10)];
    E = [E sprintf('la%d = la(%d);',i,i+1) char(10)];
    E = [E sprintf('lb%d = lb(%d);',i,i+1) char(10)];
end
E = [E 'x2 = ' sym2exp(F) char(10)];
E = [E 'fx = ' sym2exp(Fx) char(10)];
E = [E 'fu = ' sym2exp(Fu) char(10)];
E = [E 'end'];

fid = fopen(F_path,'w');
fprintf(fid,'%s',E);
fclose(fid);

%% Autogenerate loss.
L_path = fullfile(autogen_path,'autoL.m');

E = ['function [ val ] = autoL( xi, ui, i, para, dt )' char(10)];
E = [E 'x = xi(1);' char(10)];
E = [E 'y = xi(2);' char(10)];
E = [E 'u = xi(3);' char(10)];
E = [E 'l = xi(4);' char(10)];
E = [E 'v = ui(1);' char(10)];
E = [E 'p = ui(2);' char(10)];
E = [E 'val = ' sym2exp(L) char(10)];
E = [E 'end'];

fid = fopen(L_path,'w');
fprintf(fid,'%s',E);
fclose(fid);

%% Autogenerate final loss.
Lf_path = fullfile(autogen_path,'autoLf.m');

E = ['function [ val, Lfx, Lfxx ] = autoLf( x_n, i, para, x_f, del )' char(10)];
E = [E 'x = x_n(1);' char(10)];
E = [E 'y = x_n(2);' char(10)];
E = [E 'u = x_n(3);' char(10)];
E = [E 'l = x_n(4);' char(10)];
E = [E 'xf = x_f(1);' char(10)];
E = [E 'yf = x_f(2);' char(10)];
E = [E 'uf = x_f(3);' char(10)];
E = [E 'lf = x_f(4);' char(10)];
E = [E 'd1 = del(1);' char(10)];
E = [E 'd2 = del(2);' char(10)];
E = [E 'd3 = del(3);' char(10)];
E = [E 'd4 = del(4);' char(10)];
E = [E 'val = ' sym2exp(Lf) char(10)];
E = [E 'Lfx = ' sym2exp(Lfx) char(10)];
E = [E 'Lfxx = ' sym2exp(Lfxx) char(10)];
E = [E 'end'];

fid = fopen(Lf_path,'w');
fprintf(fid,'%s',E);
fclose(fid);

%% Autogenerate hamiltonian.
H_path = fullfile(autogen_path,'autoH.m');

E = ['function [H,Hx,Hxx,Hu,Hux,Huu] = autoH( xi, ui, lambda, i, para, ua, ub, la, lb, f, dt )' char(10)];
E = [E 'x = xi(1);' char(10)];
E = [E 'y = xi(2);' char(10)];
E = [E 'u = xi(3);' char(10)];
E = [E 'l = xi(4);' char(10)];
E = [E 'v = ui(1);' char(10)];
E = [E 'p = ui(2);' char(10)];
E = [E 'l1 = lambda(1);' char(10)];
E = [E 'l2 = lambda(2);' char(10)];
E = [E 'l3 = lambda(3);' char(10)];
E = [E 'l4 = lambda(4);' char(10)];
for i = 0:n_dft
    E = [E sprintf('ua%d = ua(%d);',i,i+1) char(10)];
    E = [E sprintf('ub%d = ub(%d);',i,i+1) char(10)];
    E = [E sprintf('la%d = la(%d);',i,i+1) char(10)];
    E = [E sprintf('lb%d = lb(%d);',i,i+1) char(10)];
end
E = [E 'H = ' sym2exp(H) char(10)];
E = [E 'Hx = ' sym2exp(Hx) char(10)];
E = [E 'Hxx = ' sym2exp(Hxx) char(10)];
E = [E 'Hu = ' sym2exp(Hu) char(10)];
E = [E 'Hux = ' sym2exp(Hux) char(10)];
E = [E 'Huu = ' sym2exp(Huu) char(10)];
E = [E 'end'];

fid = fopen(H_path,'w');
fprintf(fid,'%s',E);
fclose(fid);