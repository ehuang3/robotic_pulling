function data = loadFiles(folder,w,l)
  files = dir(folder);
  data.w = w;
  data.l = l;
  files(1:2) = [];
  for i=1:numel(files)
    load(strcat(folder,files(i).name));
    data.dat(i).t0 = T0;
    data.dat(i).t1 = T1;
    data.dat(i).tf = Tf;
    data.dat(i).errorbest = errorbest;
    data.dat(i).rectbest = rectbest;
    data.dat(i).xbest = xbest;
    data.dat(i).ubest = ubest;
  end
