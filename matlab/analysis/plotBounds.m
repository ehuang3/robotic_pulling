function plotBounds(data,idx)
  cp = data.dat(idx).rectbest.cp;
  alpha = atan2(cp(2,:),cp(1,:));
  poses = data.dat(idx).poses;
  poses = poses(:,1:100:size(poses,2));

  o = pi + alpha + poses(3,:);
  o1 = wrapToPi(o);
  o = [o1(1:35) o(36:end)];
  x = data.dat(idx).xbest(1:2,:);
  d1 = sqrt(x(1,:).^2 + x(2,:).^2);
  t1 = cumsum(d1);
  t1 = t1/t1(end);
  rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
  y = zeros(2,size(poses,2));
  y = poses(1:2,:);
  d2 = sqrt(y(1,:).^2 + y(2,:).^2);
  t2 = cumsum(d2);
  t2 = t2/t2(end);
  
  u = data.dat(idx).xbest(3,:);
  l = data.dat(idx).xbest(4,:);


  % plot(linspace(0,1,numel(u)),u,'r--');
  % hold on;
  % plot(linspace(0,1,numel(l)),l,'b--');
  plot(t1,u,'r--');
  hold on;
  plot(t1,l,'b--');
  % plot(linspace(0,1,numel(o)),o,'g');
  plot(t2,o,'g','LineWidth',3);
end




