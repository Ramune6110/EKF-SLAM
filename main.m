clear;
close all;
clc;

Init_parameter;

% create LM and Waypoints
[lm, wp] = create_LM_waypoints;

% State Vector [x y yaw]'
xEst = [0 0 0]';

% True State
xTrue = xEst;

PEst = zeros(3);

% Init other parameter
[veh, plines, pcount, dt, dtsum, ftag, da_table, iwp, G, data] = Init_other_parameter(DT_CONTROLS, lm, WHEELBASE, xEst, xTrue, PEst);

% gif 
if GIF_flag == 1
    filename = 'animation_sample.gif'; % Specify the output file name
    frame = getframe(gcf); % Figure 画面をムービーフレーム（構造体）としてキャプチャ
    tmp = frame2im(frame); % 画像に変更
    [A,map] = rgb2ind(tmp,256); % RGB -> インデックス画像に
    imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
end

% main loop 
while iwp ~= 0
    % Waypointsの従った角速度を計算
    [G, iwp] = compute_steering(xTrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);

    % perform loops: if final waypoint reached, go back to first
    if iwp==0 & NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end
    
    % 真値とノイズの混入した制御量を計算
    [xTrue, Vn, Gn] = get_true_value(xTrue, V, G, Q, WHEELBASE, dt);
    
    % ------ EKF SLAM --------
    % Predict
    [xEst, PEst] = predict (xEst, PEst, Vn, Gn, Q, WHEELBASE,dt);
    
    % Update
    % dt = 0.025は制御周期
    % dtの総和が観測周期を超えたら観測を行い, LMのデータ対応付けを行う
    dtsum = dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum = 0;
        % 観測条件を満たしているLMを観測し, 観測結果と見えているLMのtagを返す
        [z, ftag_visible] = get_observations(xTrue, lm, ftag, MAX_RANGE, R);
        
        % データ対応が既知か未知かで場合分け
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table] = data_associate_known(xEst,z,ftag_visible, da_table);
        else
            [zf, idf, zn] = data_associate(xEst,PEst,z,R, GATE_REJECT, GATE_AUGMENT); 
        end
        
        % EKF SLAM update
        [xEst,PEst] = update(xEst, PEst, zf, R, idf, 1); 
        
        [xEst,PEst] = augment(xEst, PEst, zn, R); 
    end
    
    % offline data store
    data = store_data(data, xEst, PEst, xTrue);
    
    %pcov = make_covariance_ellipses(xEst, PEst);
    ptmp= make_covariance_ellipses(xEst(1:3),PEst(1:3,1:3));
    pcov(:,1:size(ptmp,2))= ptmp;
    if dtsum==0
        pcount= pcount+1;
        if pcount == 5
            gcf = figure(1);
            plot(pcov(1,:), pcov(2,:), 'm'); 
            plot(data.xTrue(1,1:data.i), data.xTrue(2,1:data.i), 'r--', 'LineWidth', 1.5);
            plot(data.xEst(1,1:data.i), data.xEst(2,1:data.i), 'g--', 'LineWidth', 1.5);
            legend('LM','Trajectory', 'Waypoints', 'covariance','True', 'Estimate', 'Location', 'best');
            % Auto save graph
            saveas(gcf, 'EKF_SLAM.png');
%             figure(2);
%             imagesc(PEst)
%             colorbar
            pcount=0;
        end
%         if ~isempty(z)
%             plines = make_laser_lines (z, xEst(1:3));
%             plot(plines(1,:), plines(2,:), 'g');
%         end
    end
    
    drawnow
    
    % gif
    if GIF_flag == 1
        frame = getframe(gcf); % Figure 画面をムービーフレーム（構造体）としてキャプチャ
        tmp = frame2im(frame); % 画像に変更
        [A,map] = rgb2ind(tmp,256); % RGB -> インデックス画像に
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);% 画像をアペンド
    end
end

function [lm, wp] = create_LM_waypoints()
    % 事前に設定した経路計画に沿って走行するルートを指定
    %load('example_webmap.mat');
    load('example5.mat');
    fig = figure;
    plot(lm(1,:),lm(2,:),'b*')
    hold on, axis equal
    plot(wp(1,:),wp(2,:), 'k', wp(1,:),wp(2,:),'k.')
    xlabel('X[m]'), ylabel('Y[m]')
    set(fig, 'name', 'EKF-SLAM Simulator')
end

function [veh, plines, pcount, dt, dtsum, ftag, da_table, iwp, G, data] = Init_other_parameter(DT_CONTROLS, lm, WHEELBASE, xEst, xTrue, PEst)
    % % initialise other variables and constants
    veh      = [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
    plines   = []; % for laser line animation
    pcount   = 0;
    dt       = DT_CONTROLS; % change in time between predicts
    dtsum    = 0; % change in time since last observation
    ftag     = 1:size(lm, 2); % identifier for each landmark
    da_table = zeros(1, size(lm, 2)); % data association table 
    data     = initialise_store(xEst, PEst, xTrue); % stored data for off-line
    iwp      = 1; % index to first waypoint 
    G        = 0; % initial steer angle
end

% 初期データの格納
function data= initialise_store(xEst, PEst, xTrue)
    % offline storage initialisation
    data.i             = 1;
    data.xEst          = xEst;
    data.xTrue         = xTrue;
    data.state(1).xEst = xEst;
    data.state(1).PEst = diag(PEst);
end

% シミュレーションのデータ格納
function data= store_data(data, xEst, PEst, xTrue)
    % add current data to offline storage
    CHUNK= 5000;
    if data.i == size(data.xEst,2) % grow array in chunks to amortise reallocation
        data.xEst  = [data.xEst zeros(3,CHUNK)];
        data.xTrue = [data.xTrue zeros(3,CHUNK)];
    end
    i                  = data.i + 1;
    data.i             = i;
    data.xEst(:,i)     = xEst(1:3);
    data.xTrue(:,i)    = xTrue;
    data.state(i).xEst = xEst;
    data.state(i).PEst = diag(PEst);
end

function [xTrue, V, G] = get_true_value(xTrue, V, G, Q, WHEELBASE, dt)
    xTrue = motion_model(xTrue, V, G, WHEELBASE, dt);
    V = V + randn(1) * sqrt(Q(1,1));
    G = G + randn(1) * sqrt(Q(2,2));
end

function p = make_covariance_ellipses(x,P)
    % compute ellipses for plotting state covariances
    N= 10;
    inc= 2*pi/N;
    phi= 0:inc:2*pi;

    lenx= length(x);
    lenf= (lenx-3)/2;
    p= zeros (2,(lenf+1)*(N+2));

    ii=1:N+2;
    p(:,ii)= make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

    ctr= N+3;
    for i=1:lenf
        ii= ctr:(ctr+N+1);
        jj= 2+2*i; jj= jj:jj+1;

        p(:,ii)= make_ellipse(x(jj), P(jj,jj), 2, phi);
        ctr= ctr+N+2;
    end
end

function p= make_ellipse(x,P,s, phi)
    % make a single 2-D ellipse of s-sigmas over phi angle intervals 
    r= sqrtm(P);
    a= s*r*[cos(phi); sin(phi)];
    p(2,:)= [a(2,:)+x(2) NaN];
    p(1,:)= [a(1,:)+x(1) NaN];
end

function p= make_laser_lines (rb,xv)
    % compute set of line segments for laser range-bearing measurements
    if isempty(rb), p=[]; return, end
    len= size(rb,2);
    lnes(1,:)= zeros(1,len)+ xv(1);
    lnes(2,:)= zeros(1,len)+ xv(2);
    lnes(3:4,:)= TransformToGlobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
    p= line_plot_conversion (lnes);
end
