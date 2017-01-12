%% Test Gurobi example

import presspull.*

%% 3D Box.
Z = [0 0 0 0 1 1 1 1]';
X = [1 1 0 0 1 1 0 0]';
Y = [1 0 1 0 1 0 1 0]';
K = convhull(X,Y,Z);
% trisurf(K,X,Y,Z,'facealpha',0.5)
% axis vis3d

%%
mu = [mean(X); mean(Y); mean(Z)];
mu = rand([3,1])-0.5
A = [X'; Y'; Z'];
in_cvhull = pointInConvexHullG(mu,A)

%%

names = {'x'; 'y'; 'z'};

try
    clear model;
    model.A = sparse([1 2 3; 1 1 0]);
    model.obj = [1 1 2];
    model.rhs = [4; 1];
    model.sense = '<>';
    model.vtype = 'B';
    model.modelsense = 'max';

    clear params;
    params.outputflag = 0;
    params.resultfile = '/tmp/mip1.lp';

    result = gurobi(model, params);

    disp(result)

    for v=1:length(names)
        fprintf('%s %d\n', names{v}, result.x(v));
    end

    fprintf('Obj: %e\n', result.objval);

catch gurobiError
    fprintf('Error reported\n');
end