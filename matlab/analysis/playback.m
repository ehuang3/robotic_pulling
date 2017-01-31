function [  ] = playback( data,index)
  %PLAYBACK 
  %   
  obj = data.dat(index).rectbest;
  cp = data.dat(index).rectbest.cp;
  X = data.dat(index).xbest;
  U = data.dat(index).ubest;
  Tf = data.dat(index).tf;
  T1 = data.dat(index).t1;
  fa = 0.00;
  ea = 0.3;

  %% Playback dynamics.

  % Get object parameters.
  l = data.l;
  w = data.w;
  rect = [ l/2  -l/2  -l/2 l/2;...
    -w/2  -w/2  w/2 w/2];
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

  blue = [0 113 188]/255;
  orange = [216 82 24]/255;

  n_steps = size(X,2);
  comu = zeros([2,n_steps]);
  coml = zeros([2,n_steps]);
  tr = [];

  set(gcf,'units','inches','pos',[0 0 4 3]);
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
    if U(1,i) < 0.00001 && i < n_steps
      continue
    end
    % Plot.
    cla; hold on; axis equal; grid on; box on;
    Vo = rot(X(3,1))*V + repmat(X(1:2,1),[1,n_vert]);
    Vx = [Vo(1,K(:,1)); Vo(1,K(:,2))];
    Vy = [Vo(2,K(:,1)); Vo(2,K(:,2))];
    fill(Vx(2,:),Vy(2,:),'k','LineWidth',2,'FaceAlpha',0.0,'EdgeAlpha',1);
    Vx = [Vu(1,K(:,1)); Vu(1,K(:,2))];
    Vy = [Vu(2,K(:,1)); Vu(2,K(:,2))];
    fill(Vx(2,:),Vy(2,:),orange,'LineWidth',2,'FaceAlpha',fa,'EdgeAlpha',0.5,'EdgeColor',orange);
    tr = [tr [Vx(2,:); Vy(2,:)]];
    % hu = plot(comu(1,1:i),comu(2,1:i),'color',orange,'DisplayName','Upper Bound','LineWidth',2);
    Vx = [Vl(1,K(:,1)); Vl(1,K(:,2))];
    Vy = [Vl(2,K(:,1)); Vl(2,K(:,2))];
    fill(Vx(2,:),Vy(2,:),blue,'LineWidth',2,'FaceAlpha',fa,'EdgeAlpha',ea,'EdgeColor',blue);
    tr = [tr [Vx(2,:); Vy(2,:)]];
    % hc = plot(coml(1,1:i),coml(2,1:i),'color',blue,'DisplayName','Lower Bound','LineWidth',2);
    plot(cp(1)+xi(1),cp(2)+xi(2),'r*')
    hl = plot(X(1,1:i),X(2,1:i),'r-.','LineWidth',2,'DisplayName','Contact Point');
    % h_leg = legend([hu hc hl]);
    % set(h_leg,'FontSize',8);
    drawnow;

  end
  persist = 1:1:n_steps;
  gca;
  k = numel(1:100:size(data.dat(index).poses,2))
  col = linspace(50,255,k);
  col = [col;col;col];


  j = 0;
  for i = 1:100:size(data.dat(index).poses,2)
    j = j+1;
    rect_plot = rot(data.dat(index).poses(3,i))*rect + repmat(data.dat(index).poses(1:2,i),1,4);
    fill(rect_plot(1,:),rect_plot(2,:),'k','LineWidth',2,'FaceAlpha',fa,'EdgeAlpha',ea,'EdgeColor','k')
    j
  end
  for i = 1:numel(persist)
    Vi = rot(X(3,persist(i)))*V + repmat(X(1:2,persist(i)),[1,n_vert]);
    Vx = [Vi(1,K(:,1)); Vi(1,K(:,2))];
    Vy = [Vi(2,K(:,1)); Vi(2,K(:,2))];
    % fill(Vx(2,:),Vy(2,:),orange,'LineWidth',2,'FaceAlpha',fa,'EdgeAlpha',ea,'EdgeColor',orange);
    Vi = rot(X(4,persist(i)))*V + repmat(X(1:2,persist(i)),[1,n_vert]);
    Vx = [Vi(1,K(:,1)); Vi(1,K(:,2))];
    Vy = [Vi(2,K(:,1)); Vi(2,K(:,2))];
    % fill(Vx(2,:),Vy(2,:),blue,'LineWidth',2,'FaceAlpha',fa,'EdgeAlpha',ea,'EdgeColor',blue)
    % plot(cp(1)+X(1,persist(i)),cp(2)+X(2,persist(i)),'r*')
    % scatter(comu(1,persist(i)),comu(2,persist(i)),[],orange,'*');
    % scatter(coml(1,persist(i)),coml(2,persist(i)),[],blue,'*');
  end
  Tf_rect= rot(Tf(3))*rect + repmat(Tf(1:2),1,4);
  fill(Tf_rect(1,:),Tf_rect(2,:),'k','FaceAlpha',0.2);
  
  T1_rect= rot(T1(3))*rect + repmat(T1(1:2),1,4);
  fill(T1_rect(1,:),T1_rect(2,:),'r','FaceAlpha',0.2);

  
  load('outline_final.mat');
  top_left = cat(1,top_left.Position);
  bottom_right = cat(1,bottom_right.Position);
  plot(top_left(:,1),top_left(:,2),'color',[0.2 0.2 0.2],'LineStyle','--','Linewidth',2);
  plot(bottom_right(:,1),bottom_right(:,2),'color',[0.2 0.2 0.2],'LineStyle','--','Linewidth',2);
  xlabel('x');
  ylabel('y');


end
