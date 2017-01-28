function errorDistribution(data,idx);
  T1 = cat(2,data.dat.t1);
  Tf = cat(2,data.dat.tf);
  T1(:,idx) = [];
  Tf(:,idx) = [];

  err = T1 -Tf;

  err_r = sqrt(err(1,:).^2 + err(2,:).^2);
  avg_radial_error = mean(abs(err_r))
  err_th = abs(wrapToPi(err(3,:)));
  avg_angular_error = mean(err_th)

  [f,xi] = ksdensity(err_r);

  plot(xi,f,'LineWidth',2,'Color','b');
  ax1 = gca;
  ax1.XColor = 'b';
  ax1.YColor = 'b';

  xlabel('Distance error from commanded position');
  hold on;
  [f_max,locs] = findpeaks(f);
  [~,m] = max(f_max);
  plot([xi(locs(m)) xi(locs(m))],[f_max(m) 0],'b--');

  [f,xi] = ksdensity(err_th);
  ax1_pos = ax1.Position;
  ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','top',...
    'YAxisLocation','right',...
    'Color','None');
  ax2.XColor = 'r';
  ax2.YColor = 'r';
  line(xi,f,'Parent',ax2,'LineWidth',2,'Color','r');
  hold on;
  xlabel('Angle error from commanded position (rad)');
  [f_max,locs] = findpeaks(f);
  [~,m] = max(f_max);
  line([xi(locs(m)) xi(locs(m))],[f_max(m) 0],'Parent',ax2,'Color','r','LineStyle','--');
end


