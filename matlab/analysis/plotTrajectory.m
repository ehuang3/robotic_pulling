function [  ] = plotTrajectory(data,i)
  disp(i);
  l = data.l;
  w = data.w;
  rect = [ l/2  -l/2  -l/2 l/2;...
          -w/2  -w/2  w/2 w/2];
  rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];

  plot(data.dat(i).xbest(1,:),data.dat(i).xbest(2,:),'r','LineWidth',2);
  hold on;
  T0_rect= rot(data.dat(i).t0(3))*rect + repmat(data.dat(i).t0(1:2),1,4);
  T1_rect= rot(data.dat(i).t1(3))*rect + repmat(data.dat(i).t1(1:2),1,4);
  Tf_rect= rot(data.dat(i).tf(3))*rect + repmat(data.dat(i).tf(1:2),1,4);
  fill(T0_rect(1,:),T0_rect(2,:),'k','FaceColor','none','EdgeColor','r','LineWidth',2);
  fill(T1_rect(1,:),T1_rect(2,:),'k','FaceColor','none','EdgeColor','b','LineWidth',2);
  fill(Tf_rect(1,:),Tf_rect(2,:),'k','FaceColor','none','EdgeColor','k','LineWidth',2);
  axis equal;
  axis tight;
  xlabel('x');
  ylabel('y');

end
