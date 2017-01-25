function [ T, l ] = planCSC( s1, h1, s2, h2, rmin, step )
%PLANCSC 
%   

%% 
import ddp.CSC

%%

% cla; hold on; axis equal; grid on;
L = @(p,q) [p q];
L1 = L(s1,s1+h1);
% plot(L1(1,:),L1(2,:),'g')
% plot(s1(1),s1(2),'g*')
L2 = L(s2,s2+h2);
% plot(L2(1,:),L2(2,:),'r')
% plot(s2(1),s2(2),'r*')

% s1, p1, r1, d1, s2 ,p2, r2, d2
l = inf;
for d1 = 0:1
    for d2 = 0:1
        rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
        n1 = h1 / norm(h1);
        n2 = h2 / norm(h2);
        p1 = s1 + rmin * rot((2*d1-1)*pi/2)*n1;
        p2 = s2 + rmin * rot((2*d2-1)*pi/2)*n2;
        [Ti,li] =  CSC(s1,p1,rmin,d1,s2,p2,rmin,d2,step);
        if 0 < li && li < l
            T = Ti;
            l = li;
        end
    end
end

TT = [];
for i = 1:size(T,2)-1
    t1 = T(:,i);
    t2 = T(:,i+1);
    dt = (t2-t1)/norm(t2-t1);
    if norm(t2-t1) < 1e-7
        continue
    end
    TT = [TT, (t1+norm(h1)*dt)];
end
TT(:,end+1) = s2 + h2;
T = TT;

% plot(T(1,:),T(2,:),'k')

end








