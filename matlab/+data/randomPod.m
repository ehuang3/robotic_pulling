function [ pod ] = randomPod( n_feet, r_max, p_mean, p_sigma )
%RANDOMPOD 
%   

%% 
import presspull.*

V = [];
K = [];
area = 0;
A_max = pi*r_max^2;
A_target = A_max;
p_mean = 0.075;
p_sigma = 0.025;
%
for i = 1:n_feet
    % Uniformly sample feasible ellipses using rejection sampling.
    while true
        %%
        
        % Sample random point in r_max circle.
        r = rand;
        t = (i-1)*2*pi/n_feet + rand*2*pi/n_feet;
        pt = [sqrt(r)*cos(t); sqrt(r)*sin(t)];
        pt = r_max .* pt;
        
        % Sample random orientation.
        theta = rand*2*pi;
        a = 0.01 * [cos(theta); sin(theta)];
        b = 0.01 * [cos(theta+pi/2); sin(theta+pi/2)];
        
%         subplot(211)
        %         cla
%         hold on
%         axis equal
%         grid on
        t = linspace(0,2*pi);
%         plot(r_max*cos(t),r_max*sin(t),'k');
%         plot(r_mean*cos(t),r_mean*sin(t),'k--');
        for j = 1:n_feet
            t = j*2*pi/n_feet;
            u = [cos(t);sin(t)];
            t = intersectLineCircle([0;0],u,[0;0],r_max);
            t = max(t);
%             plot([0;t*u(1)],[0;t*u(2)],'k');
        end
%         plot(pt(1),pt(2),'k.')
%         plot([pt(1); a(1)+pt(1)],[pt(2); a(2)+pt(2)],'g')
%         plot([pt(1); b(1)+pt(1)],[pt(2); b(2)+pt(2)],'r')
        
        % Compute maximum a, b.
        q1 = (i-1)*2*pi/n_feet;
        l1 = r_max.*[cos(q1); sin(q1)];
        q2 = i*2*pi/n_feet;
        l2 = r_max.*[cos(q2); sin(q2)];
        
        as = inf([2,1]);
        ai = 1;
        [s,q] = intersectLineLine(pt,a,[0;0],l1);
        if 0 <= q && q <= 1
            a1 = pt + s.*a;
%             plot([pt(1); a1(1)],[pt(2); a1(2)],'g');
%             plot(pt(1),pt(2),'k.')
            as(ai) = norm(s.*a);
            ai = ai + 1;
        end
        [s,q] = intersectLineLine(pt,a,[0;0],l2);
        if 0 <= q && q <= 1
            a1 = pt + s.*a;
%             plot([pt(1); a1(1)],[pt(2); a1(2)],'g');
%             plot(pt(1),pt(2),'k.')
            as(ai) = norm(s.*a);
            ai = ai + 1;
        end
        [s,q] = intersectLineCircle(pt,a,[0;0],r_max);
        if all(q1 <= q & q <= q2)
            a1 = pt + s(1).*a;
            a2 = pt + s(2).*a;
%             plot([a2(1); a1(1)],[a2(2); a1(2)],'g');
%             plot(pt(1),pt(2),'k.')
            as(ai) = norm(s(1).*a);
            ai = ai + 1;
            as(ai) = norm(s(2).*a);
            ai = ai + 1;
        elseif any(q1 <= q & q <= q2)
            a3 = pt + s(q1 <= q & q <= q2).*a;
%             plot([pt(1); a3(1)],[pt(2); a3(2)],'g');
%             plot(pt(1),pt(2),'k.')
            as(ai) = norm(s(q1 <= q & q <= q2).*a);
            ai = ai + 1;
        end
        assert(ai == 3)
        as;
        
        bs = inf([2,1]);
        bi = 1;
        [s,q] = intersectLineLine(pt,b,[0;0],l1);
        if 0 <= q && q <= 1
            b1 = pt + s.*b;
%             plot([pt(1); b1(1)],[pt(2); b1(2)],'r');
%             plot(pt(1),pt(2),'k.')
            bs(bi) = norm(s.*b);
            bi = bi + 1;
        end
        [s,q] = intersectLineLine(pt,b,[0;0],l2);
        if 0 <= q && q <= 1
            b1 = pt + s.*b;
%             plot([pt(1); b1(1)],[pt(2); b1(2)],'r');
%             plot(pt(1),pt(2),'k.')
            bs(bi) = norm(s.*b);
            bi = bi + 1;
        end
        [s,q] = intersectLineCircle(pt,b,[0;0],r_max);
        if all(q1 <= q & q <= q2)
            b1 = pt + s(1).*b;
            b2 = pt + s(2).*b;
%             plot([b2(1); b1(1)],[b2(2); b1(2)],'r');
%             plot(pt(1),pt(2),'k.')
            bs(bi) = norm(s(1).*b);
            bi = bi + 1;
            bs(bi) = norm(s(2).*b);
            bi = bi + 1;
        elseif any(q1 <= q & q <= q2)
            b3 = pt + s(q1 <= q & q <= q2).*b;
%             plot([pt(1); b3(1)],[pt(2); b3(2)],'r');
%             plot(pt(1),pt(2),'k.')
            bs(bi) = norm(s(q1 <= q & q <= q2).*b);
            bi = bi + 1;
        end
        assert(bi == 3)
        bs;
        
        % Sample axes.
        % Compute bounds on the feasible axes.
        a_max = min(as);
        b_max = min(bs);
        f_l = (p_mean - p_sigma)*A_target;
        f_u = (p_mean + p_sigma)*A_target;
        a_min = f_l/(pi*b_max);
        b_min = f_l/(pi*a_max);
        
        % Stop if ellipse is infeasible.
        if a_min > a_max || b_min > b_max
            warning('Reject')
            continue
        end
        
        % Sample axes.
        n_sample = 100;
        as = rand([1,n_sample]).*(a_max - a_min) + a_min;
        bs = rand([1,n_sample]).*(b_max - b_min) + b_min;
        s_i = find(f_l <= pi.*as.*bs & pi.*as.*bs <= f_u, 1);
        
        % Stop if no feasible sample is found.
        if isempty(s_i)
            warning('No sample')
            continue
        end
        
        % Compute the probability of our sample.
%         subplot(212)
%         cla
        
        l = f_l/pi*(log(a_max) - log(a_min));
        u = (a_max-a_min)*(b_max-0);
        if f_u/(pi*b_max) <= a_max
            a_int = f_u/(pi*b_max);
            u = u - (a_max-a_int)*(b_max-0);
            %         (a_max-a_int)*(b_max-0)
            u = u + f_u/pi*(log(a_max) - log(a_int));
            %         f_u/pi*(log(a_max) - log(a_int))
%             plot(a_int,b_max,'go')
        end
        h_area = u - l;
        c_max = max(norm(l2-l1),r_max)/2; % Max chord.
        c_min = f_u/(pi*c_max); % Has to be zero.
        c_area = (c_max-c_min)^2;
        p = h_area / c_area;

        if rand < (1-p)
            warning('low probability')
            continue
        end
        
%         hold on
%         axis auto
%         grid on
%         plot([a_min a_min a_max a_max a_min], [b_min b_max b_max b_min b_min],'k')
        a = linspace(a_min, a_max);
        b = f_l./(pi.*a);
%         plot(a,b,'b');
        b = f_u./(pi.*a);
%         plot(a,b,'b');

        a = as(s_i);
        b = bs(s_i);
%         plot(a,b,'k.')
        t = linspace(0,2*pi);
        rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
        pts = rot(theta) * [a*cos(t); b*sin(t)];
        pts = pts + repmat(pt, [1,length(t)]);
%         subplot(211)
%         hold on
%         plot(pts(1,:),pts(2,:),'b')

        area = area + pi*a*b;

        % Save ellipse.
        n_t = length(t);
        k = [(1:n_t-1)' (2:n_t)'];
        V = [V pts];
        if isempty(K)
            K = k;
        else
            K = [K; max(K(:)) + k];
        end

        break
    end
end

% Construct output.
pod = struct;
pod.V = V;
pod.K = K;
pod.area = area;
pod.com = [0;0];

end

