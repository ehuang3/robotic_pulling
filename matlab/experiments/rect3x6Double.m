%% Exp double bound dynamics.
clc
clear
clear import

dynamics_namespace = 'double';

import presspull.*
import data.*
import(['ddp.dynamics.' dynamics_namespace '.autoF']);
import(['ddp.dynamics.' dynamics_namespace '.autoL']);
import(['ddp.dynamics.' dynamics_namespace '.autoLf']);
import(['ddp.dynamics.' dynamics_namespace '.autoH']);
import(['ddp.dynamics.' dynamics_namespace '.ddpDouble']);
import ddp.dft
import ddp.playback
import ddp.idft
import ddp.computeRearAxle
import ddp.planCSC

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SYSTEM PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Workspace bounds.
x_work_min = 0;
x_work_max = 0.91;
y_work_min = 0;
y_work_max = 0.61;
work = struct('xmin',x_work_min,'xmax',x_work_max,'ymin',y_work_min,'ymax',y_work_max);

% A-B target locations. Robot will pull between A and B.
A = [0.15; 0.30];
B = [0.75; 0.30];
AB_side = 0.05;
A_theta = [-pi/2 pi/2];
B_theta = [-pi/2 pi/2];

% 3x6 rectangle.
rect_w = 0.0762;
rect_l = 0.1524;
mass = 0.188;

% List of contact points relative to object frame.
cp1 = [0; 0];
cp2 = [0.03819; 0];
cp3 = [0.07612; 0];
cp4 = [0.11395; 0];
cp5 = [0; 0.03800];
cp6 = [0.03819; 0.03800];
cp7 = [0.07612; 0.03800];
cp8 = [0.11395; 0.03800];
CP = [cp1 cp2 cp3 cp4 cp5 cp6 cp7 cp8];
cpoff = [0.01508 + 0.0042; 0.01508 + 0.0042];
CP = CP + repmat(cpoff,[1,8]);
CP = CP + repmat([-rect_l/2;-rect_w/2],[1,8]);
contact_point_list = CP;

% Percentage of object always in contact with surface.
percent_in_contact = 0.50;

% Plot.
do_plot = 1;

%% Initialize dynamic system for rectangles x contacts.
n_cp = size(contact_point_list,2);
for i = 1:n_cp
    % Robot.
    g = 9.807;
    zforce = g * 0.0;
    % Create rectangle.
    rect = rectangle(rect_l/2, rect_w/2);
    % Add contact point.
    rect.cp = contact_point_list(:,i);
    % Interpolate CoM.
    total_mass = mass + zforce / g;
    rect.com = (mass/total_mass)*rect.com + (zforce/g/total_mass)*rect.cp;
    % Compute bounds.
    R = fillScanLines2DGrid2(rect.K,rect.V(1,:),rect.V(2,:),2.5e-3);
    rect.R = R;
    rect.LB = zeros([1,size(R,2)]);
    rect.UB = 1 / (percent_in_contact * size(R,2)) * ones([1,size(R,2)]);
    % rect.UB = ones([1,size(R,2)]);
    [L,U,T] = computeBounds(rect, rect.cp);
    rect.L = L;
    rect.U = U;
    rect.T = T;
    % Fit bounds.
    f = 1/(2*pi);
    [ua,ub] = dft(rect.U,rect.T,f,100);
    [la,lb] = dft(rect.L,rect.T,f,100);
    rect.ua = ua;
    rect.ub = ub;
    rect.la = la;
    rect.lb = lb;
    % Add to rectangle x contacts list.
    rectxcp(i) = rect;
end

%% Plot bounds.
if do_plot
    figure(1);
    clf;
    subplot(4,4,[13 14])
    for i = 1:length(rectxcp)
        rect = rectxcp(i);
        cla; hold on; grid on;
        plot(rect.T,rect.U)
        plot(rect.T,rect.L)
        title('bounds')
        xlabel('\theta')
        ylabel('\theta''')
        axis tight
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% READ MOCAP AND ROBOT POSES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting pose.
T0 = [  2*AB_side*(rand-0.5) + A(1) ; ...
        2*AB_side*(rand-0.5) + A(2) ; ...
        rand*(A_theta(2) - A_theta(1)) + A_theta(1)];

% TODO: Angle needs to be in [0, 2pi).

% Randomized end poses.
% Target pose.
dA = norm(T0(1:2)-A);
dB = norm(T0(1:2)-B);
C = B;
if dB < dA
    C = A;
end
T1 = [  2*AB_side*(rand-0.5) + C(1) ; ...
        2*AB_side*(rand-0.5) + C(2) ; ...
        rand*(B_theta(2) - B_theta(1)) + B_theta(1)];

%% Get a rectangle.
fmin = inf;
xbest = []; ubest = [];
ibest = 0; errorbest = [];
for i = 1:length(rectxcp)
    rect = rectxcp(i);
    [xnom,unom,valid,ferror] = ddpDouble(rect,work,param,T0,T1,do_plot);
    if ~valid
        continue
    end
    if norm(ferror(1:4)) < fmin
        fmin = norm(ferror(1:4));
        xbest = xnom;
        ubest = unom;
        ibest = i;
        errorbest = ferror;
    end
end
errorbest

%% Object poses.
rect = rectxcp(ibest);
rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
V = rect.V; n_v = size(V,2);
com = rect.com;
cp = rect.cp;
V0 = rot(T0(3))*V + repmat(T0(1:2),[1,n_v]);
com0 = rot(T0(3))*com + T0(1:2);
cp0 = rot(T0(3))*cp + T0(1:2);
V1 = rot(T1(3))*V + repmat(T1(1:2),[1,n_v]);
com1 = rot(T1(3))*com + T1(1:2);
cp1 = rot(T1(3))*cp + T1(1:2);
% Plot controls.
if do_plot
    figure(1);
    subplot(4,4,[3 4])
    cla; hold on; grid on; axis tight;
    plot(xbest(3,:),'-')
    plot(xbest(4,:),'-')
    title('u l')
    ylabel('radians');
    subplot(4,4,8)
    cla; hold on; grid on; axis tight;
    stem(ubest(2,:),'b.')
    title('\phi'); ylabel('radians')
    subplot(4,4,7)
    cla; hold on; grid on; axis tight;
    stem(ubest(1,:),'r.')
    title('\nabla d'); ylabel('m')
end
% Playback solution.
if do_plot
    figure(1);
    subplot(4,4,[1 2 5 6 9 10]);
    playback(rect,cp,xbest,ubest,[]);
    XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
    hold on; grid on;
    plot([0 0 work_l work_l 0],[0 work_w work_w 0 0],'k.-')
    XY = XYfunc(V0,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'b');
    plot(cp0(1),cp0(2),'b.');
    plot(cp0(1),cp0(2),'b.');
    XY = XYfunc(V1,rect.K);
    plot(XY(1:2,:),XY(3:4,:),'r');
    plot(cp1(1),cp1(2),'r.');
    title('workspace'); xlabel('x'); ylabel('y')
    axis equal
    axis([0 work_l 0 work_w]);
end






