function indices = findExclusion(data)

 T0 = cat(2,data.dat.t0);
 T1 = cat(2,data.dat.t1);
 Tf = cat(2,data.dat.tf);
 rect_center = zeros(2,100);

 indices = [];
 rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
 for i = 1:numel(data.dat)
 % If trajectory endpoint and T1 are not the same, then delete point
   traj_endpoint = data.dat(i).xbest(1:2,end);
   rect_center(:,i) = abs(traj_endpoint - (T1(1:2,i) + rot(T1(3,i))*data.dat(i).rectbest.cp));
   if norm(rect_center(:,i))>0.010
     indices = [indices i];
   end
   if norm(T1(1:2,i)-Tf(1:2,i))>0.02
     if ~(indices(end) == i)
       indices = [indices i];
     end
   end
 end
end

