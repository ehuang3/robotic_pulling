function errorDistribution(data,idx);

%%
  T1 = cat(2,data.dat.t1);
  Tf = cat(2,data.dat.tf);
  dat = cat(2,data.dat.rectbest);
  cp = cat(2,dat.cp);
  xend = [];
  for i = 1:length(data.dat)
      xend(:,i) = data.dat(i).xbest(:,end);
  end
  T1(:,idx) = [];
  Tf(:,idx) = [];
  cp(:,idx) = [];
  xend(:,idx) = [];
  
  alpha = atan2(cp(2,:),cp(1,:));
  
  err_cmd = wrapToPi(Tf(3,:)+alpha-pi) - wrapToPi(mean(xend(3:4,:)))
  err_cmd_u = wrapToPi(Tf(3,:)+alpha-pi) - wrapToPi(xend(3,:))
  err_cmd_l = wrapToPi(Tf(3,:)+alpha-pi) - wrapToPi(xend(4,:))

  err = T1 -Tf;

  err_r = sqrt(err(1,:).^2 + err(2,:).^2);

  %Compute mean and std dev
  avg_radial_error = mean(abs(err_r))
  std_radial_error = std(abs(err_r))
  err_th = abs(wrapToPi(err(3,:)));
  avg_angular_error = rad2deg(mean(abs(err_th)))
  std_angular_error = rad2deg(std(abs(err_th)))
  
  avg_cmd_error = rad2deg(mean(abs(err_cmd)))
  std_cmd_error = rad2deg(std(abs(err_cmd)))

  % 

  %%
  % Check if angular error is within bounds:
  
  d = data.dat;
  d(idx) = [];
  err_th_bound_ref = atan2(cp(2,:),cp(1,:)) + Tf(3,:) + repmat(pi, 1,size(Tf,2));
  eidx = zeros(1,numel(d));
  for i=1:numel(d)
    flag = false;
    if d(i).xbest(3,end)<err_th_bound_ref(i)
      e = rad2deg(abs(d(i).xbest(3,end)-err_th_bound_ref(i)));
      fprintf('%d: upper limit out of bounds by %d degrees\n',i,e);
      flag = true;
    end
    if d(i).xbest(4,end)>err_th_bound_ref(i)
      e = rad2deg(abs(d(i).xbest(3,end)-err_th_bound_ref(i)));
      fprintf('%d: upper limit out of bounds by %d degrees\n',i,e);
      flag = true;
    end
    if flag
      eidx(i) = 1;
    end
  end
  fprintf('Total Number of experiments out of bounds: %d', sum(eidx));

    

  %Plot the distribution
  figure(1)
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


