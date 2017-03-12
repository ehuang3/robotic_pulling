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
%Experiment constants:
num_experiments = 100;
data_location = '~/src/presspull/data/experiment/';
experiment_prefix = 'small_rect8cp_100_mocap';
% Workspace bounds.
x_work_min = 0.39;
x_work_max = 0.70;
y_work_min =-0.076;
y_work_max = 0.35;
reachable.xmin = 0.26;
reachable.xmax = 0.70;
reachable.ymin = -0.24;
reachable.ymax = 0.450;
work = struct('xmin',x_work_min,'xmax',x_work_max,'ymin',y_work_min,'ymax',y_work_max);

% A-B target locations. Robot will pull between A and B.
A = [0.15; 0.30];
B = [0.75; 0.30];
AB_side = 0.05;
A_theta = [-pi/2 pi/2];
B_theta = [-pi/2 pi/2];

% 3x6 rectangle.
rect_w = 0.0496;
rect_l = 0.07465;
mass = 0.064;

% List of contact points relative to object frame.
% cp1 = [0; 0];
% cp2 = [0.03819; 0];
% cp3 = [0.07612; 0];
% cp4 = [0.11395; 0];
% cp5 = [0; 0.03800];
% cp6 = [0.03819; 0.03800];
% cp7 = [0.07612; 0.03800];
% cp8 = [0.11395; 0.03800];
%List of contact points for small rect:
cp1 = [0.019315;0];
cp2 = -cp1;
%List of contact points for small acrylic rect:
cp1 = [-0.0314525;0.019];
cp2 = [-0.0314525;0];
cp3 = [-0.0314525;-0.019];
cp4 = [0;0.019];
cp5 = -cp4;
cp6 = -cp3;
cp7 = -cp2;
cp8 = -cp1;
CP = [cp1 cp2 cp3 cp4 cp5 cp6 cp7 cp8];
%cpoff = [0.01508 + 0.0042; 0.01508 + 0.0042];
%CP = CP + repmat(cpoff,[1,8]);
%CP = CP + repmat([-rect_l/2;-rect_w/2],[1,size(CP,2)]);
contact_point_list = CP;

% Percentage of object always in contact with surface.
percent_in_contact = 0.25;

% Object Frame
%object_transform = [        0.934847617898597        -0.354817300112436         0.012830231885651          79.9554719491778;
%                             0.0133231301968747      -0.00105403389144328        -0.999910687618806          2.54721157684607;
%                             0.354799134033717         0.934935063281658       0.00374191601290136         -82.4080912060258;
%                                             0                         0                         0                         1;];
%Large Rectangle Transform
% object_transform = [    0.9997    0.0216    0.0111   43.4023;
%                         0.0111    0.0023   -0.9999    2.0325;
%                        -0.0216    0.9998    0.0021 -106.1857;
%                              0         0         0    1.0000;];
%Small Rectangle Transform
object_transform = [  0.9998    0.0179    0.0091   16.6952;
                      0.0090    0.0009   -1.0000    2.0177;
                     -0.0179    0.9998    0.0008  -44.6577;
                           0         0         0    1.0000];
global mocap_data;
% Plot.
do_plot = 1;

% Robot Height Parameters:
safe_z = 470;
interact_z = 435;

if do_plot
    figure(1); clf; hold on; axis equal;
    rect = rectangle(rect_l/2, rect_w/2);
    V = rect.V + repmat([rect_l/2;rect_w/2],[1 5]);
    plot(V(1,:),V(2,:),'k')
    set(gca,'xtick',[],'ytick',[])
    xlabel('75mm')
    ylabel('50mm')
    CPP = CP + repmat([rect_l/2;rect_w/2],[1,8]);
    for i = 1:size(CP,2)
        rad = 0.003;
        cpp = CPP(:,i);
        wt = linspace(0,2*pi);
        plot(rad*cos(wt) + cpp(1), rad*sin(wt) + cpp(2), 'k')
    end
    axis([0 rect_l 0 rect_w])
    
    set(gcf, 'Color', 'w');
    set(gca,'fontsize',14)
    export_fig src/presspull/tex/fig/acrylic_rect.eps 
end

%% Initialize dynamic system for rectangles x contacts.
try
    load('/home/eric/src/presspull/data/rectxcp_8cp_small.mat')
catch
    %%
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
%% INITIALIZE ROS COMMUNICATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    rosinit
end
%%
mocap = rossubscriber('/Mocap');
robot = robotSubscriber();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START EXPERIMENT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting pose.

for N=2:num_experiments
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% READ MOCAP AND ROBOT POSES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T0 = getObjectPose2D(mocap,object_transform)'
    
    T0(3) = T0(3) + pi/2;
    rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
    offset = [rect_l/2; rect_w/2];
    T0(1:2) = T0(1:2) + rot(T0(3))*offset;

    % T0(1) = T0(1) - rect_w/2
    % T0(2) = T0(2) + rect_l/2

    %
    % T0 = [  2*AB_side*(rand-0.5) + A(1) ; ...
    %         2*AB_side*(rand-0.5) + A(2) ; ...
    %         rand*(A_theta(2) - A_theta(1)) + A_theta(1)];

    % TODO: Angle needs to be in [0, 2pi).

    % Randomized end poses.
    % Target pose.
    dA = norm(T0(1:2)-A);
    dB = norm(T0(1:2)-B);
    C = B;
    if dB < dA
        C = A;
    end
    %T1 = [  2*AB_side*(rand-0.5) + C(1) ; ...
%             2*AB_side*(rand-0.5) + C(2) ; ...
%             rand*(B_theta(2) - B_theta(1)) + B_theta(1)];
    %T1 = [0.45; 0.25; pi]
    inworkspace = 0;
    while ~inworkspace
        inworkspace = 1;
        T1 = [(x_work_max-x_work_min)*0.7*rand + x_work_min;
              (y_work_max-y_work_min)*0.7*rand + y_work_min;
              2*pi*rand;];
        T1_rect_points = rot(-T1(3))*[-rect_l -rect_l rect_l rect_l;-rect_w rect_w -rect_w rect_w]*0.5;
        T1_rect = repmat(T1(1:2),1,4) + T1_rect_points;
        if any(T1_rect(1,:)>x_work_max) | any(T1_rect(1,:)<x_work_min) | ...
                any(T1_rect(2,:)>y_work_max) | any(T1_rect(2,:)<y_work_min)
            inworkspace = 0;
            disp('failed');
        end
    end
            
        
    % Object poses.
    rect = rectxcp(1);
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

    % Plot start and end poses.
    if do_plot
        XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
        figure(1);
        subplot(4,4,[1 2 5 6 9 10])
        cla; hold on; grid on;
        xn = work.xmin; xm = work.xmax; yn = work.ymin; ym = work.ymax;
        plot([xn xn xm xm xn],[yn ym ym yn yn],'k.-')
        XY = XYfunc(V0,rect.K);
        plot(XY(1:2,:),XY(3:4,:),'b');
        plot(cp0(1),cp0(2),'b.');
        plot(cp0(1),cp0(2),'b.');
        XY = XYfunc(V1,rect.K);
        plot(XY(1:2,:),XY(3:4,:),'r');
        plot(cp1(1),cp1(2),'r.');
        title('workspace'); xlabel('x'); ylabel('y')
        axis equal
    %     axis([xn xm yn ym]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% DDP PARAMETERS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Slope of pseudo-Huber loss - [x, y, u, l]
    param.k = [1000 * [5 5 1 1] 40];

    % Convergence width of pseudo-Huber loss - [x, y, u, l]
    param.p = [0.01 0.01 0.02 0.02 0.0];

    % Max DDP iterations.
    param.max_iters = 300;

    % Step-size for Dubin's paths.
    param.dubin_step = 1.0e-2;

    % Steering angle for computing Dubin's paths.
    param.steering_angle = 45/180 * pi;

    %% Get a rectangle.
    fmin = inf;
    xbest = []; ubest = [];
    ibest = 0; errorbest = [];
    for i = 1:length(rectxcp)
        rect = rectxcp(i);
        [xnom,unom,valid,ferror] = ddpDouble(rect,reachable,param,T0,T1,do_plot);
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
    rectbest=rectxcp(ibest);
    if isempty(xbest)
        continue
    end

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
        cla;
        playback(rect,cp,xbest,ubest,[]);
        XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
        hold on; grid on;
        plot([x_work_min x_work_min x_work_max x_work_max x_work_min],[y_work_min y_work_max y_work_max y_work_min y_work_min],'k.-')
        XY = XYfunc(V0,rect.K);
        plot(XY(1:2,:),XY(3:4,:),'b');
        plot(cp0(1),cp0(2),'b.');
        plot(cp0(1),cp0(2),'b.');
        XY = XYfunc(V1,rect.K);
        plot(XY(1:2,:),XY(3:4,:),'r');
        plot(cp1(1),cp1(2),'r.');
        title('workspace'); xlabel('x'); ylabel('y')
        axis equal
        axis([x_work_min x_work_max y_work_min y_work_max]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% RUN TRAJECTORY ON ROBOT
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Convert Trajectory to row major and in millimeters
    %
    trajectory = 1000*xbest(1:2,:)';
    sendAndExecuteTrajectoryRealtime(trajectory,robot,safe_z,interact_z);
    poses = mocapToPose(mocap_data,object_transform);



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% GET FINAL POSE FROM MOCAP
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    final_pose = getObjectPose2D(mocap,object_transform);
    Tf = getObjectPose2D(mocap,object_transform)';

    Tf(3) = Tf(3) + pi/2;
    rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
    offset = [rect_l/2; rect_w/2];
    Tf(1:2) = Tf(1:2) + rot(Tf(3))*offset;
    % Object poses.
    rect = rectxcp(1);
    rot = @(t) [cos(t) -sin(t); sin(t) cos(t)];
    V = rect.V; n_v = size(V,2);
    com = rect.com;
    cp = rect.cp;
    Vf = rot(Tf(3))*V + repmat(Tf(1:2),[1,n_v]);
    comf = rot(Tf(3))*com + Tf(1:2);
    cpf = rot(Tf(3))*cp + Tf(1:2);
    V1 = rot(T1(3))*V + repmat(T1(1:2),[1,n_v]);
    com1 = rot(T1(3))*com + T1(1:2);
    cp1 = rot(T1(3))*cp + T1(1:2);

    if do_plot
        XYfunc = @(Vin,Kin) [Vin(1,Kin(:,1)); Vin(1,Kin(:,2)); Vin(2,Kin(:,1)); Vin(2,Kin(:,2))];
        figure(1);
        subplot(4,4,[1 2 5 6 9 10])
        %cla;
        hold on; grid on;
        %xn = work.xmin; xm = work.xmax; yn = work.ymin; ym = work.ymax;
        %plot([xn xn xm xm xn],[yn ym ym yn yn],'k.-')
        XY = XYfunc(Vf,rect.K);
        plot(XY(1:2,:),XY(3:4,:),'b');
        plot(cpf(1),cpf(2),'b.');
        plot(cpf(1),cpf(2),'b.');

        title('workspace'); xlabel('x'); ylabel('y')
        axis equal
    %     axis([xn xm yn ym]);
    end
    %Save Data:
    save(strcat(data_location,experiment_prefix,'_',num2str(N),'.mat'),'T0','Tf','T1','xbest','ubest','rectbest','errorbest','poses');
end    

