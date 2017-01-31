function data = loadFiles(folder,w,l)
  files = dir(folder);
  data.w = w;
  data.l = l;
  rect = [ l/2  -l/2  -l/2 l/2;...
          -w/2  -w/2  w/2 w/2];
  files(1:2) = [];
  rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
  for i=1:numel(files)
    load(strcat(folder,files(i).name));
    data.dat(i).t0 = T0;
    data.dat(i).t1 = T1;
    data.dat(i).tf = Tf;
    data.dat(i).errorbest = errorbest;
    data.dat(i).rectbest = rectbest;
    data.dat(i).xbest = xbest;
    data.dat(i).ubest = ubest;
    for j=1:size(poses,2)
      poses(3,j) = poses(3,j) + pi/2;
      offset = [l/2; w/2];
      poses(1:2,j) = poses(1:2,j) + rot(poses(3,j))*offset;
    end
    data.dat(i).poses = poses;

  end
