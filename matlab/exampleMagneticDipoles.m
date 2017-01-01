u0 = 1;
p = 5;
l = 0.05;
l1 = l*[1;0;0];
l2 = -l*[0;1;0];
l3 = l*[-1;0;0];
l4 = -l*[0;-1;0];
L = [l1,l2,l3,l4];
mu1 = p*l1;
mu2 = p*l2;
mu3 = p*l3;
mu4 = p*l4;
Mu = [mu1,mu2,mu3,mu4];
r = 0.10;
r1 = r*[1;0;0];
r2 = r*[0;1;0];
r3 = r*[-1;0;0];
r4 = r*[0;-1;0];
R = [r1,r2,r3,r4];

% Force exterted by m1 on m2.
F = @(m1,m2,r,rh) (3*u0)./(4*pi*norm(r).^4).*(cross(cross(rh,m1),m2) + cross(cross(rh,m2),m1) - 2.*rh.*dot(m1,m2) + 5.*rh.*dot(cross(rh,m1),cross(rh,m2)));

T = zeros([3,4]);
for i = 1:4
    for j = 1:4
        if i == j
            continue
        end
        % m1 m2
        m2 = Mu(:,i);
        m1 = Mu(:,j);
        % r
        r2 = R(:,i);
        r1 = R(:,j);
        r = r2 - r1;
        % rh
        rh = r ./ norm(r);
        % F
        T(:,i) = T(:,i) + F(m1,m2,r,rh);
    end
end

T