function [ in_cvhull ] = pointInConvexHullG( pt, X, Y, Z )
%POINTINCONVEXHULLG Gurobi version.
%   

%% Point in convex hull.
import presspull.*
% Reduce the dimensionality of X by taking its convex hull.
X = [X; Y; Z];
% K = convhull(X');
% X = X(:,K);

% Translate the center of X to the origin.
mu = mean(X,2);
X = X - repmat(mu,[1 size(X,2)]);
pt = pt - mu;

b = ones([size(X,2) 1]);

[~,fval] = glinprog(-pt,X',b);
in_cvhull = -fval <= 1;

% % Solve for level set...
% try
%     clear model;
%     model.A = sparse(X');
%     model.obj = pt';
%     model.rhs = b;
%     model.sense = '<';
%     model.vtype = 'B';
%     model.modelsense = 'max';
% 
%     clear params;
%     params.outputflag = 0;
%     params.resultfile = '/tmp/mip1.lp';
% 
%     result = gurobi(model, params);
% 
% %     disp(result)
% %     disp(result.x)
% %     disp(result.slack)
% 
% %     for v=1:length(names)
% %         fprintf('%s %d\n', names{v}, result.x(v));
% %     end
% 
%     fprintf('Obj: %e\n', result.objval);
%     
%     in_cvhull = result.objval <= 1;
% 
% catch gurobiError
%     fprintf('Error reported\n');
%     gurobiError
% end

end

