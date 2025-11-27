function JetAcker_Final_Fix()
    % [Step 0] 清理与初始化
    try 
        % 尝试发送一次停车指令再退出旧程序
        tempNode = ros2node("/safety_stop_" + string(randi(9999)));
        tempPub = ros2publisher(tempNode, "/cmd_vel", "geometry_msgs/Twist");
        stopMsg = ros2message(tempPub);
        send(tempPub, stopMsg); clear tempNode tempPub;
    catch
    end
    delete(timerfindall); clear; clc;

    % [Step 1] 参数配置
    pars = struct(...
        'HFOV', deg2rad(58.4), ...
        'targetDist', 1.0, ...     % 目标距离 1.0米
        'stopDist', 0.6, ...       % 刹车距离 0.6米
        'maxSpd', 0.35, ...
        'turnGain', 4.0, ...
        'skipFrames', 2);          % 跳帧优化

    % ROS 连接
    setenv("ROS_DOMAIN_ID","0");
    node = ros2node("/final_tracker_" + string(randi(999)));
    
    % 订阅与发布
    subCam = ros2subscriber(node, "/depth_cam/rgb/image_raw/compressed", "sensor_msgs/CompressedImage", "Reliability","besteffort");
    subScan = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan", "Reliability","besteffort");
    cmdPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
    cmdMsg = ros2message(cmdPub);

    % 加载 YOLO (自动回退)
    try det = yolov8ObjectDetector('yolov8n'); catch, det = yolov8ObjectDetector('yolov8s'); end
    
    % 界面
    fig = figure('Name', 'JetAcker Final Fix', 'CloseRequestFcn', @cleanUp, ...
                 'NumberTitle', 'off', 'Position', [100, 100, 800, 600], ...
                 'KeyPressFcn', @keyPress, 'KeyReleaseFcn', @keyRelease); % 添加键盘监听
    ax = axes('Parent', fig);
    hIm = imshow(zeros(480,640,'uint8'), 'Parent', ax);
    
    % 状态数据
    fig.UserData = struct('v', 0, 'w', 0, 'run', true, 'manual_override', false);
    
    % 启动定时器
    t = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
              'TimerFcn', @(t,e) sendCmd(cmdPub, cmdMsg, fig));
    start(t);

    % [Step 4] 主循环
    tracker = struct('active', false, 'u', 0, 'v', 0, 'lostTime', 0);
    last_bbox = []; loopCount = 0;
    
    while isvalid(fig) && fig.UserData.run
        imgMsg = subCam.LatestMessage;
        scanMsg = subScan.LatestMessage;
        if isempty(imgMsg), pause(0.01); continue; end
        
        img = rosReadImage(imgMsg);
        [H, W, ~] = size(img); cx = W/2;
        
        % --- 1. 视觉处理 (带跳帧) ---
        loopCount = loopCount + 1;
        if mod(loopCount, pars.skipFrames+1) == 1
            [bboxes, ~, labels] = detect(det, img, 'Threshold', 0.5);
            isPerson = (string(labels) == "person");
            if any(isPerson)
                % 简单的追踪逻辑：找离中心最近的
                pbs = bboxes(isPerson, :);
                centers = pbs(:,1) + pbs(:,3)/2;
                [~, pick] = min(abs(centers - cx));
                last_bbox = pbs(pick, :);
                tracker.active = true;
                tracker.lostTime = 0;
            else
                if tracker.active % 丢失缓冲
                    if tracker.lostTime == 0, tracker.lostTime = tic; end
                    if toc(tracker.lostTime) > 3.0, tracker.active = false; last_bbox = []; end
                else
                    last_bbox = [];
                end
            end
        end
        
        % --- 2. 核心修复：更鲁棒的雷达融合 ---
        rawDist = NaN; 
        rawAng = 0;
        statusInfo = "SEARCHING";
        boxColor = 'blue';
        
        if tracker.active && ~isempty(last_bbox)
            % 视觉角度 (左正右负? 需根据摄像头实际安装调整，通常左+右-)
            % 假设摄像头: x向右为正. 
            u_center = last_bbox(1) + last_bbox(3)/2;
            rawAng = -(u_center - cx) / (W/2) * (pars.HFOV/2); % 注意这里的负号，视实际情况而定
            
            if ~isempty(scanMsg)
                ranges = scanMsg.ranges;
                angles = scanMsg.angle_min + (0:numel(ranges)-1)' * scanMsg.angle_increment;
                
                % 【关键修复】角度归一化处理 (解决 0~2pi 和 -pi~pi 的匹配问题)
                % 目标: 在雷达数据中找到和 rawAng 最接近的角度
                % 技巧: 将所有角度差异限制在 0~pi 之间
                
                % 雷达通常前方是0度。视觉rawAng也是相对于前方。
                % 直接计算最小角度差
                angDiff = abs(atan2(sin(angles - rawAng), cos(angles - rawAng)));
                
                [minDiff, rIdx] = min(angDiff);
                
                % 只有当角度匹配误差小于 5度 (0.08弧度) 时才认为有效
                if minDiff < 0.15 
                    dVal = ranges(rIdx);
                    if isfinite(dVal) && dVal > 0.1
                        rawDist = dVal;
                    end
                end
            end
            
            % 状态判断
            if isnan(rawDist)
                statusInfo = "LIDAR LOST (NaN)";
                boxColor = 'magenta'; % 紫色代表视觉看到了，但雷达没跟上
            elseif rawDist < pars.stopDist
                statusInfo = "TOO CLOSE - BRAKE";
                boxColor = 'red';     % 红色代表太近
            else
                statusInfo = sprintf("TRACKING (%.2fm)", rawDist);
                boxColor = 'green';   % 绿色代表正常
            end
            
            % 画框
            img = insertObjectAnnotation(img, 'rect', last_bbox, ...
                sprintf("D:%.2fm", rawDist), 'Color', boxColor, 'LineWidth', 3, 'FontSize', 18);
        end
        
        % --- 3. 控制决策 ---
        calc_v = 0; calc_w = 0;
        
        % 如果按下键盘 W，强制接管 (Manual Override)
        if fig.UserData.manual_override
            calc_v = 0.2; 
            statusInfo = "KEYBOARD OVERRIDE";
        elseif tracker.active && ~isnan(rawDist)
            % 转向 (P控制)
            calc_w = pars.turnGain * rawAng; % 注意方向符号
            
            % 前进
            if rawDist > pars.targetDist
                calc_v = pars.maxSpd;
                % 弯道减速
                calc_v = calc_v * max(0, 1.0 - abs(calc_w)/2.0);
            elseif rawDist < pars.stopDist
                calc_v = 0;
            end
        end
        
        % 更新全局变量
        fig.UserData.v = calc_v;
        fig.UserData.w = calc_w;
        
        hIm.CData = img;
        title(ax, sprintf("[%s] Cmd: v=%.2f", statusInfo, calc_v), 'FontSize', 14);
        drawnow limitrate;
    end
    
    % --- 键盘回调 (按住W强制前进) ---
    function keyPress(src, event)
        if strcmp(event.Key, 'w')
            src.UserData.manual_override = true;
        end
    end
    function keyRelease(src, event)
        if strcmp(event.Key, 'w')
            src.UserData.manual_override = false;
        end
    end

    function sendCmd(pub, msg, fig)
        if ~isvalid(fig), return; end
        d = fig.UserData;
        % 阿克曼防卡死
        final_v = d.v;
        if abs(d.w) > 0.1 && d.v < 0.05 && ~d.manual_override
            final_v = 0.12; 
        end
        msg.linear.x = double(final_v);
        msg.angular.z = double(d.w);
        send(pub, msg);
    end

    function cleanUp(~,~)
        % 强制发 0
        try
            node = ros2node("/stop_guard_" + string(randi(999)));
            pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
            msg = ros2message(pub);
            for i=1:5, send(pub, msg); pause(0.02); end
        catch
        end
        delete(timerfindall);
        disp("✅ 强制刹车完成，程序退出");
    end
end