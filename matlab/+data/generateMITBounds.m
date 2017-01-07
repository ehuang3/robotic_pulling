%% Generate bounds for all MIT Objects.
clc
clear

import presspull.*
import data.*

Objects = loadMITObjects;
keys = Objects.keys;

for i_obj = 7:8
    
    %% Pick an object.
    obj = Objects(keys{i_obj})
    
    V0 = obj.V;
    K = obj.K;
    com0 = obj.com;
    
    %% Generate a list of contact points on the boundary.
    % Compute total length of the boundary.
    n_V = length(K);
    d = 0;
    for i = 1:n_V
        A = V0(:,K(i,1));
        B = V0(:,K(i,2));
        d = d + norm(B-A);
    end
    
    % Generate list of contact points.
    n_cp = 30;
    CP0 = zeros([2,n_cp]);
    cp_step = d / n_cp;
    i = 1;
    j = 1;
    k = 0;
    b = 0;
    while i <= n_cp
        A = V0(:,K(j,1));
        B = V0(:,K(j,2));
        t = (i*cp_step-b)/norm(B-A);
        assert(0 <= t);
        if t > 1 + 1e-7
            b = b + norm(B-A);
            j = j + 1;
            continue
        end
        cp = (1-t)*A + t*B;
        CP0(:,i) = cp;
        i = i + 1;
    end

%     % Plot object.
%     cla
%     hold on
%     plot(V0(1,:),V0(2,:),'b');
%     plot(CP0(1,:),CP0(2,:),'r.');
    
    %% Pick a contact point.
    for i_cp = 1:n_cp
        cp0 = CP0(:,i_cp)
        
        %% Pick a bound type.
        for i_bt = 1:2
            % Reset V0 and K.
            V0 = obj.V;
            K = obj.K;
            if i_bt == 1
                % Using exact bound.
                btype = [];
            else
                % Using Peshkin's bound.
                btype = 'p';
                % Find furthest vertex from CoM.
                r = 0;
                for i = 1:size(V0,2)
                    d = norm(V0(:,i)-com0);
                    if d > r
                        r = d;
                    end
                end
                % Create circumcircle.
                t = linspace(0,2*pi,361);
                V0 = repmat(com0,[1,361]) + r.*[cos(t); sin(t)];
                K = [(1:360)' (2:361)'];
            end
            
            %% Translate contact point to origin and re-orient object.
            step = 3e-3;
            Rg0 = fillScanLines2DGrid2(K,V0(1,:),V0(2,:),step);
            
            % Shift contact point to origin and rotate object into 0 orientation.
            n_v = size(V0,2);
            d = com0 - cp0;
            t = -atan2(d(2),d(1));
            R = [cos(t) -sin(t); sin(t) cos(t)];
            V = V0 - repmat(cp0,1,n_v);
            V = R * V;
            com = R * (com0 - cp0);
            cp = cp0 - cp0;
            Rg = R * (Rg0 - repmat(cp0,1,size(Rg0,2)));
            
%             % Plot object.
%             figure(3)
%             subplot(211)
%             hold on
%             cla;
%             axis equal
%             grid on
%             plot(V0(1,:),V0(2,:));
%             plot(com0(1),com0(2),'k*');
%             plot(cp0(1),cp0(2),'r*');
%             plot(Rg0(1,:),Rg0(2,:),'g.')
%             subplot(212)
%             hold on
%             cla;
%             axis equal
%             grid on
%             plot(V(1,:),V(2,:));
%             plot(com(1),com(2),'k*');
%             plot(cp(1),cp(2),'r*');
%             plot(Rg(1,:),Rg(2,:),'g.')

            %% Compute exact angular velocity bound.
            R = @(t) [cos(t) -sin(t); sin(t) cos(t)];
            display('computing bounds')
            tic
            t = linspace(-pi/2,pi/2,100);
            B = zeros([2 length(t)]);
            U = zeros([1 length(t)]);
            L = zeros([1 length(t)]);
            for i = 1:length(t)
                Vt = R(t(i)) * V;
                comt = R(t(i)) * com;
                Rgt = R(t(i)) * Rg;
                [xl, xu] = computeRotationCenterExtrema(Rgt,comt(1),comt(2),Vt,K);
                U(i) = -1./xu;
                L(i) = -1./xl;
            end
            toc
            
            %% Save exact bounds to file.
            % Make bounds directory.
            data_path = getDataPath();
            out_dir = fullfile(data_path, 'bounds', obj.name);
            mkdire(out_dir);
            out_file = fullfile(out_dir, [sprintf('%02d', i_cp) btype '.mat'])
            
            % Save bounds to file.
            S = struct;
            S.obj = obj;
            S.obj.V = V;
            S.obj.R = Rgt;
            S.obj.com = com;
            S.obj.cp = cp;
            S.obj.t = t;
            S.obj.U = U;
            S.obj.L = L;
            save(out_file,'-struct','S');
            
        end
        
    end
    
end
