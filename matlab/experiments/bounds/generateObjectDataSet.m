%% Generate a dataset of objects.
clc
clear
clear import

import data.*
import presspull.*

%% MIT objects.
mit_objects = loadMITObjects;

% Create objects + contact point set.
n_cp = 10;
i = 1;
clear objects
for m = 1:length(mit_objects)
    % Object parameters.
    obj = mit_objects(m);
    V = obj.V;
    K = obj.K;
    com = obj.com;
    name = obj.name;
    R = fillScanLines2DGrid2(K,V(1,:),V(2,:),4e-3);
    size(R)
    % 10 contact points, press down 30%
    cp = getSpacedContactPoints(V,K,n_cp);
    for n = 1:size(cp,2)
        objects(i).V = V;
        objects(i).K = K;
        objects(i).com = 2/3 * com + 1/3 * cp(:,n);
        objects(i).cp = cp(:,n);
        objects(i).name = name;
        objects(i).R = R;
        i = i + 1;
    end
end

%% Save path.
data_path = getDataPath;
bounds_path = fullfile(data_path,'bounds');
mit_bounds_file = fullfile(bounds_path,'mit_bounds.mat');

%% Bounds.
clear bounds;
for i = 1:length(objects)
    obj = objects(i)
    % Exact bounds.
    [L,U,T] = computeBounds(obj,obj.cp);
    exact = struct;
    exact.L = L;
    exact.U = U;
    exact.T = T;
    % 50 percent contact.
    obj50 = obj;
    n_R = size(obj50.R,2);
    obj50.LB = zeros([1,n_R]);
    obj50.UB = 1/(0.5 * n_R) .* ones([1,n_R]);
    [L,U,T] = computeBounds(obj50,obj50.cp);
    fifty = struct;
    fifty.L = L;
    fifty.U = U;
    fifty.T = T;
    % Peshkins.
    objp = obj;
    Vp = objp.V;
    comp = objp.com;
    rp = max(sqrt(sum((Vp-repmat(comp,[1,size(Vp,2)])).^2)));
    s = linspace(0,2*pi,100);
    Vp = [rp*cos(s)+comp(1); rp*sin(s)+comp(2)];
    Kp = [(1:99)' (2:100)'];
    Rp = fillScanLines2DGrid2(Kp,Vp(1,:),Vp(2,:),5e-3);
    objp.V = Vp;
    objp.K = Kp;
    objp.R = Rp;
    [L,U,T] = computeBounds(objp,objp.cp);
    peshkin.L = L;
    peshkin.U = U;
    peshkin.T = T;
    % Store bounds.
    obj.exact = exact;
    obj.fifty = fifty;
    obj.peshkin = peshkin;
    bounds(i) = obj;
    % Save.
    save(mit_bounds_file,'bounds');
end

%% 
r_max = 0;
Objects = loadMITObjects
keys = Objects.keys;
r = zeros([1, length(keys)]);
for i = 1:length(keys)
    obj = Objects(keys{i});
    V = obj.V;
    K = obj.K;
    com = obj.com;
    for j = 1:size(V,2)
        A = V(:,j);
        d = norm(com-A);
        if r(i) < d
            r(i) = d;
        end
    end
    if r(i) > r_max
        r_max = r(i);
    end
end
r_max
r_mean = mean(r)