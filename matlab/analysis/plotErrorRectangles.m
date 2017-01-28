function plotErrorRectangles(data,eIndices)
  %plot rectangles:
  figure()
  l = data.l;
  w = data.w;
  rect = [ l/2  -l/2  -l/2 l/2;...
          -w/2  -w/2  w/2 w/2];
  rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
  for i=1:numel(data.dat)
    e = data.dat(i).tf-data.dat(i).t1;
    new_rect= rot(e(3))*rect + repmat(e(1:2),1,4);
    if any(i == eIndices)
      % fill(new_rect(1,:),new_rect(2,:),'b','FaceAlpha',0.2,'EdgeAlpha',0.2);
      continue
    else
      fill(new_rect(1,:),new_rect(2,:),'k','FaceAlpha',0.1,'EdgeAlpha',0.8);
    end
    hold on;
  end
  fill(rect(1,:),rect(2,:),'r','FaceAlpha',0.0,'EdgeColor','r');
  axis equal;
  xlabel('x');
  ylabel('y');
end
