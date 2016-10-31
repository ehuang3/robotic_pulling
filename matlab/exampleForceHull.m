%% Example frictional force hull.
import presspull.*

% Generate tetrapod.
[X, Y, K] = generate2DTetrapod(0.5,0.7,0);
X = X + 0.5;

theta = linspace(pi/2-0.02,-pi/2+0.02);
for t = theta
    % Rotate tetrapod.
    rot = [cos(t) -sin(t); sin(t) cos(t)];
    X_t = rot(1,1).*X + rot(1,2).*Y;
    Y_t = rot(2,1).*X + rot(2,2).*Y;

    % Fill scanlines.
    R = fillScanLines2D(K,X_t,Y_t);
    R_x = R(:,1);
    R_y = R(:,2);
    num_pts = size(R,1);
    P = ones([num_pts,1]) ./ num_pts;
    
    % Compute frictional force.
    ff = computeFrictionalForce( mu, xr, -sign(xr), R, P );

    % Compute center.
    [xr, mf] = computeRotationCenter(R,P);

    if mf > 1e-7
        continue
    end

    % Compute Fx(R).
    f0 = 1;
    mu = 1;
    F_x = Fx(R,xr,-sign(xr),mu);
    K_R = convhull(R_x,R_y,F_x);
    
    % Plot Fx(R).
    subplot(221)
    hold off
    fill(X_t, Y_t, 'k');
    hold on
    axis square
    grid on
    
    trimesh(K_R,R_x,R_y,F_x,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
    xlabel('x')
    ylabel('y')
    zlabel('f_x')
    view(3)
    
    % Compute Fy(R).
    f0 = 1;
    mu = 1;
    F_y = Fy(R,xr,-sign(xr),mu);
    K_R = convhull(R_x,R_y,F_y);
    
    % Plot Fy(R).
    subplot(223)
    hold off
    fill(X_t, Y_t, 'k');
    hold on
    axis square
    grid on
    
    trimesh(K_R,R_x,R_y,F_y,'facealpha',0.5,'facecolor','interp','edgecolor','interp')
    xlabel('x')
    ylabel('y')
    zlabel('f_y')
    view(3)
    
    % Compute Fx and ff intersection.
    n = [0 0 1]';
    d = ff(1);
    [x0, y0, k0] = intersectPlaneConvexHull(n,d,R_x,R_y,F_x);
    xy0 = inv(rot) * [x0';y0'];
    x0 = xy0(1,:)';
    y0 = xy0(2,:)';
    
    % Plot fx intersection.
    subplot(222)
    hold off
    plot(X(K),Y(K),'b')
    hold on
    plot(x0(k0),y0(k0),'r')
    plot(0.5,0,'k*')
    axis equal
    grid on
    title(['f_x = ' num2str(d)])

    % Compute Fy and ff intersection.
    n = [0 0 1]';
    d = ff(2);
    [x1, y1, k1] = intersectPlaneConvexHull(n,d,R_x,R_y,F_y);
    xy1 = inv(rot) * [x1';y1'];
    x1 = xy1(1,:)';
    y1 = xy1(2,:)';

    % Plot fy intersection.
    subplot(224)
    hold off
    plot(X(K),Y(K),'b')
    hold on
    plot(x1(k1),y1(k1),'r')
    plot(0.5,0,'k*')
    axis equal
    grid on
    title(['f_y = ' num2str(d)])

    pause(0.1)

end