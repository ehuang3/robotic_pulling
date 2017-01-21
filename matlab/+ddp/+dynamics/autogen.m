%% 
clc
clear
clear import

dynamics_namespace = 'pointR';

import data.*
import ddp.dynamics.sym2exp
import ddp.dynamics.autofunc
import(['ddp.dynamics.' dynamics_namespace '.symF']);
import(['ddp.dynamics.' dynamics_namespace '.symL']);
import(['ddp.dynamics.' dynamics_namespace '.symLf']);
import(['ddp.dynamics.' dynamics_namespace '.symH']);

dynamics_folder = fullfile(fileparts(getDataPath),'matlab','+ddp','+dynamics',['+' dynamics_namespace]);

% Get variables.
vars_func = ['ddp.dynamics.' dynamics_namespace '.vars'];
eval(vars_func)

% Symbolic functions.
[xnew,Fx,Fu] = symF(x,u);
[L] = symL(x,u);
[Lf,Lfx,Lfxx] = symLf(x,xf,p,k);
[H,Hx,Hxx,Hu,Hux,Huu] = symH(x,u,lambda);
O = struct('xnew',xnew,'Fx',Fx,'Fu',Fu,'L',L,'Lf',Lf,'Lfx',Lfx,'Lfxx',Lfxx, ...
    'H',H,'Hx',Hx,'Hxx',Hxx,'Hu',Hu,'Hux',Hux,'Huu',Huu);

% File paths.
F_path = fullfile(dynamics_folder,'autoF.m');
L_path = fullfile(dynamics_folder,'autoL.m');
Lf_path = fullfile(dynamics_folder,'autoLf.m');
H_path = fullfile(dynamics_folder,'autoH.m');

% Write to files.
autofunc(F_path,tempF,V,O)
autofunc(L_path,tempL,V,O)
autofunc(Lf_path,tempLf,V,O)
autofunc(H_path,tempH,V,O)