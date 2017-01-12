%% Generate random 2,3,4-pods.
clc
clear

import presspull.*
import data.*

%% Compute maximum circumcircle radius across all MIT objects.
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

%% Generate 2,3,4-pods.

data_path = getDataPath;
pod_path = fullfile(data_path, 'pods')
mkdire(pod_path);

n_pods = 30;
n_feet = 3;
p_total = 0.10;
p_total_sigma = 0.025;
p_foot = p_total / n_feet;
p_sigma = p_total_sigma / n_feet;

pod_file = [num2str(n_feet) 'pod_' sprintf('%0.2f',p_total) '+' sprintf('%0.3f', p_total_sigma) '.mat']

%% Generate pods.
clear pods
i = 1;
while true
    %% Sample pod.
    p = randomPod(n_feet, r_max, p_foot, p_sigma);
    
    %% Sample CoM.
    sample_CoM = 1;
    if sample_CoM
        V = p.V;
        K = convhull(V');
        x = V(1,K);
        y = V(2,K);
%         hold on
%         plot(x,y,'b')
        x_min = min(x); y_min = min(y); x_max = max(x); y_max = max(y);
        while true
            x0 = rand*(x_max-x_min)+x_min;
            y0 = rand*(y_max-y_min)+y_min;
            if pointInConvexHull2D([x0;y0],x,y)
                break;
            end
        end
        p.com = [x0;y0];
    end

    %% Plot pod.
    V = p.V;
    K = p.K;
    x = [V(1,K(:,1)); V(1,K(:,2))];
    y = [V(2,K(:,1)); V(2,K(:,2))];
%     cla
%     axis equal; grid on; hold on;
%     plot(x,y,'b');
    t = linspace(0,2*pi);
%     plot(r_max*cos(t),r_max*sin(t),'k');
%     plot(p.com(1),p.com(2),'b.');
%     pause(1)
    
    % Add pod.
    pods(i) = p;
    i = i + 1;
    if i > n_pods
        break
    end
end

%% Save pods to file.
Objects = pods;
save(fullfile(pod_path,pod_file), 'Objects')
