function [  ] = playback( obj, cp, X, U, t )
%PLAYBACK 
%   

%% Playback dynamics.

% Get object parameters.
V0 = obj.V;
K = obj.K;
com0 = obj.com;
cp0 = cp;

% Translate contact point to origin and re-orient object.
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
n_vert = size(V0,2);
d = com0 - cp0;
t0 = -atan2(d(2),d(1));
V = V0 - repmat(cp0,1,n_vert);
V = rot(t0) * V;
com = rot(t0) * (com0 - cp0);
cp = cp0 - cp0;

% % Plot.
% subplot(221)
% cla; hold on; axis equal; grid on;
% Vx = [V(1,K(:,1)); V(1,K(:,2))];
% Vy = [V(2,K(:,1)); V(2,K(:,2))];
% plot(Vx,Vy,'k');
% plot(com(1),com(2),'k*');
% plot(cp(1),cp(2),'r*');
% 
blue = [0 113 188]/255;
orange = [216 82 24]/255;
% 
% % Plot.
% subplot(2,2,3)
% cla; hold on; axis auto; grid on;
% u = X(3,:) + pi - U(2,:);
% l = X(4,:) + pi - U(2,:);
% plot(t,u,'color',orange)
% plot(t,l,'color',blue)
% 
% % Plot.
% dt = t(2) - t(1);
% alpha = (u(2:end)-u(1:end-1))./dt./U(1,1:end-1);
% beta = (l(2:end)-l(1:end-1))./dt./U(1,1:end-1);
% subplot(224)
% cla; hold on; axis auto; grid on;
% plot(t(1:end-1),alpha,'color',orange)
% plot(t(1:end-1),beta,'color',blue)
% 
% % Plot.
% subplot(221)
% cla;hold on; axis auto; grid on;
% plot(u(1:end-1),alpha,'color',orange)
% plot(l(1:end-1),beta,'color',blue)
% title('phase')
% xlabel('\theta')
% % axis([-4 4 -4 4])

%% 
figure(3)

n_steps = size(X,2);
comu = zeros([2,n_steps]);
coml = zeros([2,n_steps]);
for i = 1:n_steps
    %% Draw object.
    % Compute poses.
    xi = X(1:2,i);
    ui = X(3,i);
    li = X(4,i);
    Vu = rot(ui) * V + repmat(xi,[1,n_vert]);
    Vl = rot(li) * V + repmat(xi,[1,n_vert]);
    comu(:,i) = rot(ui) * com + xi;
    coml(:,i) = rot(li) * com + xi;
    % Plot.
%     subplot(222);
    cla; hold on; axis equal; grid on;
    Vo = rot(X(3,1))*V + repmat(X(1:2,1),[1,n_vert]);
    Vx = [Vo(1,K(:,1)); Vo(1,K(:,2))];
    Vy = [Vo(2,K(:,1)); Vo(2,K(:,2))];
    plot(Vx,Vy,'k')
    Vx = [Vu(1,K(:,1)); Vu(1,K(:,2))];
    Vy = [Vu(2,K(:,1)); Vu(2,K(:,2))];
    plot(Vx,Vy,'color',orange)
    plot(comu(1,1:i),comu(2,1:i),'color',orange)
    Vx = [Vl(1,K(:,1)); Vl(1,K(:,2))];
    Vy = [Vl(2,K(:,1)); Vl(2,K(:,2))];
    plot(Vx,Vy,'color',blue)
    plot(coml(1,1:i),coml(2,1:i),'color',blue)
    plot(cp(1)+xi(1),cp(2)+xi(2),'r*')
    plot(X(1,1:i),X(2,1:i),'r-.')
    pause(0.1)
end


end