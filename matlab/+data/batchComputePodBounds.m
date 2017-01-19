%% Compute pod bounds.
clc
clear

import presspull.*
import data.*

%% Settings
num_contacts = 15;
objects_path = '/home/eric/src/presspull/data/pods/3pod_0.10+0.025.mat';
output_path = '/home/eric/src/presspull/data/bounds/pods/3pod_0.10+0.025';

mkdire(output_path);

%% Load object.
load('/home/eric/src/presspull/data/pods/3pod_0.10+0.025.mat')

%% Loop.
% i = 1;
for i = 24:30
    object = Objects(i);
    disp(['Object ' num2str(i)])
    disp(object)
    
    %% Get contact points.
    V0 = object.V;
    K = object.K;
    cps = getSpacedContactPoints(V0,K,num_contacts);
    
    %% Pick a contact point.
    clear result
    for j = 1:length(cps)
        contact_point = cps(:,j);
        disp(['Contact ' num2str(j)]);
        
        %% Compute exact angular velocity bounds.
        disp('    Exact bounds');
        [L,U,T] = computeBounds(object,contact_point);
        
        result(j).object = object;
        result(j).object.L = L;
        result(j).object.U = U;
        result(j).object.T = T;
        result(j).object.cp = contact_point;
        
        %% Compute Peshkin's bounds.
        V0 = object.V;
        com0 = object.com;
        r = max(sqrt(sum((V0-repmat(com0,[1,size(V0,2)])).^2)));
        peshkin = object;
        s = linspace(0,2*pi);
        peshkin.V = [r*cos(s)+com0(1); r*sin(s)+com0(2)];
        peshkin.K = [(1:99)' (2:100)'];
        
        disp('    Peshkin''s bounds');
        [L,U,T] = computeBounds(peshkin,contact_point);
        
        result(j).peshkin = peshkin;
        result(j).peshkin.L = L;
        result(j).peshkin.U = U;
        result(j).peshkin.T = T;
        result(j).peshkin.cp = contact_point;
    end

    %% Save results to file.
    result_path = fullfile(output_path, [sprintf('%02d',i) '.mat']);
    save(result_path, 'result');
    
    disp(' ')
end