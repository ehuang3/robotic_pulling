function [ traj, l_total ] = CSC( s1, p1, r1, d1, s2 ,p2, r2, d2, step )
%CSC Summary of this function goes here
%   Detailed explanation goes here

%% 
import presspull.*
import ddp.*

% blue = [0 113 188]/255;
% orange = [216 82 24]/255;

% sanitize inputs
swap = r2 > r1;
if swap
    [r2 r1] = deal(r1, r2);
    [p2 p1] = deal(p1, p2);
end
if abs(r1-r2) < 1e-7
    r1 = r1 + 1e-7;
end

T = [];

C = @(r,p,w) [r*cos(w); r*sin(w)] + repmat(p,[1,100]);
L = @(p,q) [p q];
w = linspace(0,2*pi);
C1 = C(r1,p1,w);
C2 = C(r2,p2,w);
% cla; hold on; axis equal; grid on;
% plot(C1(1,:),C1(2,:),'k')
% plot(C2(1,:),C2(2,:),'k')

V1 = p2-p1;
D = norm(V1);
p3 = p1 + V1/2;
r3 = D/2;
C3 = C(r3,p3,w);
L1 = L(p1,p2);
% plot(C3(1,:),C3(2,:),'b')
% plot(p3(1),p3(2),'b*')
% plot(L1(1,:),L1(2,:),'color',orange)

r4 = r1 + r2;
C4 = C(r4,p1,w);
% plot(C4(1,:),C4(2,:),'g')

loc = @(a,b,c) acos((b^2 + c^2 - a^2)/(2*b*c));
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];

% t1
gamma = loc(r3,r4,r3);
V2 = rot(gamma)* (r4 * V1 / norm(V1));
pt = p1 + V2;
% plot(pt(1),pt(2),'k.')

pt1 = p1 + r1 * V2 / norm(V2);
% plot(pt1(1),pt1(2),'r.')

pt2 = pt1 + (p2-pt);
L2 = L(pt1,pt2);
% plot(L2(1,:),L2(2,:),'m')

if swap && isreal(gamma)
    T = [T pt2 pt1];
elseif isreal(gamma)
    T = [T pt1 pt2];
end

% t2
gamma = -loc(r3,r4,r3);
V2 = rot(gamma)* (r4 * V1 / norm(V1));
pt = p1 + V2;
% plot(pt(1),pt(2),'k.')

pt1 = p1 + r1 * V2 / norm(V2);
% plot(pt1(1),pt1(2),'r.')

pt2 = pt1 + (p2-pt);
L2 = L(pt1,pt2);
% plot(L2(1,:),L2(2,:),'m')

if swap && isreal(gamma)
    T = [T pt2 pt1];
elseif isreal(gamma)
    T = [T pt1 pt2];
end

% t3
r4 = r1 - r2;
C4 = C(r4,p1,w);
% plot(C4(1,:),C4(2,:),'g')

gamma = loc(r3,r4,r3);
V2 = rot(gamma)* (r4 * V1 / norm(V1));
pt = p1 + V2;
% plot(pt(1),pt(2),'k.')

pt1 = p1 + r1 * V2 / norm(V2);
% plot(pt1(1),pt1(2),'r.')

pt2 = pt1 + (p2-pt);
L2 = L(pt1,pt2);
% plot(L2(1,:),L2(2,:),'m')

if swap && isreal(gamma)
    T = [T pt2 pt1];
elseif isreal(gamma)
    T = [T pt1 pt2];
end

% t4
gamma = -loc(r3,r4,r3);
V2 = rot(gamma)* (r4 * V1 / norm(V1));
pt = p1 + V2;
% plot(pt(1),pt(2),'k.')

pt1 = p1 + r1 * V2 / norm(V2);
% plot(pt1(1),pt1(2),'r.')

pt2 = pt1 + (p2-pt);
L2 = L(pt1,pt2);
% plot(L2(1,:),L2(2,:),'m')

if swap && isreal(gamma)
    T = [T pt2 pt1];
elseif isreal(gamma)
    T = [T pt1 pt2];
end

% 
for i = 1:2:size(T,2)
    % plot(T(1,i),T(2,i),'ro')
end

% plot(s1(1),s1(2),'g*')
% plot(s2(1),s2(2),'r*')

% 
match = false;
traj = [];
l_total = 0;
for i = 1:2:size(T,2)
    t1 = T(:,i);
    t2 = T(:,i+1);
    t = atan2(t1(2)-p1(2),t1(1)-p1(1));
    % left hand tangent
    si = dot([-sin(t); cos(t)],t2-t1);
    if (si > 0 && ~d1) || (si < 0 && d1)
        continue
    end
    t = atan2(t2(2)-p2(2),t2(1)-p2(1));
    si = dot([-sin(t); cos(t)],t2-t1);
    if (si > 0 && ~d2) || (si < 0 && d2)
        continue
    end
    match = true;

    % angle to rotate
    [l,tN] = arcLength(s1-p1,t1-p1,d1);
    t0 = atan2(s1(2)-p1(2),s1(1)-p1(1));
    w = linspace(t0,t0+tN);
    C1 = C(r1,p1,w);
    % plot(C1(1,:),C1(2,:),'g')
    
    l_total = l;

    % trajectory segment
    w = linspace(t0,t0+tN, round(l/step));
    traj = repmat(p1,[1,round(l/step)]) + [r1*cos(w);r1*sin(w)];

    L2 = L(t1,t2);
    % plot(L2(1,:),L2(2,:),'g')
    t = linspace(0,1,round(norm(t2-t1)/step));
    S = repmat(1-t,[2,1]).*repmat(t1,[1,round(norm(t2-t1)/step)]) + repmat(t,[2,1]).*repmat(t2,[1,round(norm(t2-t1)/step)]);
    traj = [traj S];

    l_total = l_total + norm(t2-t1);

    [l,tN] = arcLength(t2-p2,s2-p2,d2);
    t0 = atan2(t2(2)-p2(2),t2(1)-p2(1));
    w = linspace(t0,t0+tN);
    C3 = C(r2,p2,w);
    % plot(C3(1,:),C3(2,:),'g')
    
    w = linspace(t0,t0+tN, round(l/step));
    C3 = repmat(p2,[1,round(l/step)]) + [r2*cos(w);r2*sin(w)];
    traj = [traj C3];
    
    l_total = l_total + l;

end
assert(match || size(T,2)<8)
l_total;






end

