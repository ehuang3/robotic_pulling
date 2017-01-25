%% 

T = rect.T;
L = rect.L;
U = rect.U;
subplot(211)
cla; hold on; grid on;
plot(T,L)
plot(T,U)
for i = 1:126%length(rect.T)
    if T(i) < 0
        continue
    end
    if abs(U(i)) < 1e-7
        continue
    end
    V = rect.V;
    K = rect.K;
    com = rect.com;
    cp = rect.cp;
    subplot(212)
    cla; hold on; axis equal; grid on;
    plot(V(1,:),V(2,:),'k')
    plot(cp(1),cp(2),'r*')
    plot(com(1),com(2),'k*')
    t = -T(i);
    dp = 0.05 * [cos(t); sin(t)];
    line = @(p,q) [p q];
    pull = line(cp,cp+dp);
    plot(pull(1,:),pull(2,:),'g')
    xr = -1/U(i);
    rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
    xa = rot(-pi/2) * dp / norm(dp);
    xr = xr * xa;
    turn = line(cp,xr+cp);
    plot(turn(1,:),turn(2,:),'r')

    nc = (cp-com)/norm(cp-com);
    ic = xr + cp;
    rac = com + (ic'*nc)*nc;
    rmin = norm(ic-rac);
    plot(rac(1),rac(2),'r.')
end

%% 








