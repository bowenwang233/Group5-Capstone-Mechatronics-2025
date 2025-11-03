% SensorFusion.m
% Live fusion of YOLOv8 "person" detections (camera) with 2D LiDAR range.
% - Uses geometric intersection to find the distance along the visualized ray.
% - Includes the fix for the 'bb' variable error.
clear; clc;
%% --- ROS 2 Setup ---
setenv("ROS_DOMAIN_ID","0");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");
nodeName = "/matlab_sensor_fusion";
if ~isempty(ros2node("list")) && any(strcmp(ros2node("list"), nodeName))
    n = ros2node(nodeName); % reuse
else
    n = ros2node(nodeName);
end
% Camera subscriber
imgTopic = "/depth_cam/rgb/image_raw/compressed";
subCam = ros2subscriber(n, imgTopic, "sensor_msgs/CompressedImage", ...
    Reliability="besteffort", History="keeplast", Depth=50);
% LiDAR subscriber
scanTopic = "/scan";
subScan = ros2subscriber(n, scanTopic, "sensor_msgs/LaserScan", ...
    Reliability="besteffort", History="keeplast", Depth=10);
% YOLOv8 (MATLAB) detector
det = yolov8ObjectDetector('yolov8s');
%% --- Camera Intrinsics (from FOV only for bearing mapping) ---
HFOV_deg = 58.4;              % horizontal FOV in degrees
HFOV = deg2rad(HFOV_deg);
yaw_offset_deg = 0;           % tweak if cam vs lidar misaligned
yaw_offset = deg2rad(yaw_offset_deg);
% The camera is 7cm (0.07m) BEHIND the LiDAR's origin (0,0) on the plot.
CAM_Y_OFFSET_M = -0.07;
%% --- Figure/UI Setup: image + LiDAR plot ---
fig = figure('Name', 'Sensor Fusion: YOLO (person) + LiDAR', ...
             'NumberTitle','off', 'Position', [100,100,1200,520]);
% Right: camera image
axImg = subplot(1,2,2);
hIm  = imshow(uint8(zeros(480,640,3)), 'Parent', axImg);
title(axImg, sprintf('Camera (%s)', imgTopic));
hold(axImg, 'on');
hCenter = plot(axImg, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
hLabel  = text(axImg, 10, 20, '', 'Color','y','FontSize',12,'FontWeight','bold');
hold(axImg, 'off');
% Left: LiDAR top-down view
axLidar = subplot(1,2,1);
hold(axLidar, 'on'); grid(axLidar, 'on'); axis(axLidar, 'equal');
title(axLidar, sprintf('LiDAR Front View (CW 90°) — %s', scanTopic));
xlabel(axLidar, 'X right (m)');
ylabel(axLidar, 'Y forward (m)');
xlim(axLidar, [-5 5]); ylim(axLidar, [-5 5]);
hScan = scatter(axLidar, NaN, NaN, 10, 'b', 'filled');
hRay  = plot(axLidar, [0 0], [0 0], 'y-', 'LineWidth', 2);
hold(axLidar, 'off');
disp("🚀 Streaming & fusing… Close the window to stop.");
%% --- Main Loop ---
while isvalid(fig)
    [imgMsg, ok] = receive(subCam, 0.2);
    scanMsg = subScan.LatestMessage;
    if ok
        img = rosReadImage(imgMsg);
        [H, W, ~] = size(img);
        cx = W/2;
        [bboxes, scores, labels] = detect(det, img, 'Threshold', 0.8);
        isPerson = (string(labels) == "person");
        imgOut = img;
        set(hCenter,'XData',NaN,'YData',NaN);
        set(hLabel,'String','');
        
        if ~isempty(scanMsg)
            ranges = double(scanMsg.ranges(:));
            N  = numel(ranges);
            th0 = double(scanMsg.angle_min);
            dth = double(scanMsg.angle_increment);
            th  = th0 + (0:N-1).' * dth;
            mask = isfinite(ranges) & ranges > double(scanMsg.range_min);
            
            % Plot transformation (90° clockwise: (x,y) -> (y, -x))
            x_plot =  ranges .* sin(th);
            y_plot = -ranges .* cos(th);
            set(hScan, 'XData', x_plot(mask), 'YData', y_plot(mask));
            
            if any(mask)
                rmax = min(max(ranges(mask))*1.1, 10);
                xlim(axLidar, [-rmax rmax]); ylim(axLidar, [-rmax rmax]);
            end
        else
            mask = []; x_plot = []; y_plot = [];
        end
        
        if any(isPerson)
            % --- FIX FOR 'bb' ERROR IS HERE ---
            % This block correctly selects the highest-confidence person
            % and defines 'bb' before it is used.
            pb = bboxes(isPerson, :);
            ps = scores(isPerson);
            [~, pick] = max(ps);
            bb = pb(pick, :);
            
            imgOut = insertObjectAnnotation(imgOut, 'rectangle', bb, ...
                       "person: "+string(round(ps(pick),2)), ...
                       'Color','yellow','LineWidth',3);
            % --- END OF FIX ---

            % This calculates the angle, it will NOT be changed.
            u_c = bb(1) + bb(3)/2;
            v_c = bb(2) + bb(4)/2;
            x_norm   = (u_c - cx) / (W/2);
            thetaCam = x_norm * (HFOV/2);
            thetaLid = thetaCam - yaw_offset;
            
            % This draws the ray, it will NOT be changed.
            Rvis = 5;
            x_line =  Rvis * sin(thetaLid);
            y_line =  Rvis * cos(thetaLid);
            set(hRay, 'XData', [0 x_line], 'YData', [0 y_line]);

            % Use the geometric intersection function to find the distance FROM THE LIDAR.
            intersection_threshold_m = 0.05; % 5 cm tolerance
            r_from_lidar = findRayIntersection(x_plot(mask), y_plot(mask), thetaLid, intersection_threshold_m);

            if ~isnan(r_from_lidar)
                % Now, account for the 7cm offset to find the true distance FROM THE CAMERA.
                % We use the Law of Cosines: a^2 = b^2 + c^2 - 2bc*cos(A)
                % a = distance from camera to point
                % b = distance from lidar to point (r_from_lidar)
                % c = distance from lidar to camera (CAM_Y_OFFSET_M)
                % A = angle at the lidar
                angle_at_lidar = pi/2 - thetaLid; % Angle relative to the plot's Y-axis
                
                dist_from_camera_sq = r_from_lidar^2 + CAM_Y_OFFSET_M^2 - ...
                                      2 * r_from_lidar * abs(CAM_Y_OFFSET_M) * cos(angle_at_lidar);
                
                dist_from_camera = sqrt(dist_from_camera_sq);
                
                % Display the NEW, corrected distance.
                set(hCenter, 'XData', u_c, 'YData', v_c);
                set(hLabel, 'String', sprintf('%.2f m @ %.1f°', dist_from_camera, rad2deg(thetaCam)), ...
                            'Position', [u_c+8, max(10, v_c-10)]);
                imgOut = insertText(imgOut, [bb(1), max(1,bb(2)-24)], ...
                           "dist: "+sprintf('%.2f m', dist_from_camera), ...
                           'FontSize',18,'BoxOpacity',0.6, ...
                           'TextColor','black','BoxColor','yellow');
            end
        end
        set(hIm, 'CData', imgOut);
    end
    drawnow limitrate;
    pause(0.01);
end
disp("Viewer closed, script stopped.");
clear n subCam subScan

%% === Local Helper Function (must be last) ===
function [range_at_intersection] = findRayIntersection(x_points, y_points, target_angle, threshold_m)
% Finds a robust distance where the plotted ray intersects the LiDAR points.
% Coordinate system: your plotted frame (X = right, Y = forward; front is more negative Y in your CW-rotated plot).
% target_angle is the SAME angle you used for the yellow bearing line.
%
% Robust steps:
%  1) Build ray direction in plotted coords: d = [-sin(theta), cos(theta)]  (matches how you draw the line)
%  2) Keep only points "near" the ray: |det(d,p)| < threshold_m   (perp distance band)
%  3) Keep only points AHEAD of the robot along the ray: dot(p,d) > 0
%  4) Robustly filter ranges with MAD + trimmed mean (10–90%) ; fallback to min if few points.

    range_at_intersection = NaN;
    if isempty(x_points) || isempty(y_points)
        return;
    end

    % Column vectors
    x = x_points(:);
    y = y_points(:);

    % --- Ray direction in plotted coordinates (consistent with your bearing line) ---
    % Your bearing line uses: x_line = -R*sin(theta), y_line =  R*cos(theta).
    % So unit direction vector:
    dx = -sin(target_angle);
    dy =  cos(target_angle);
    % (No need to re-normalize; sin/cos already unit-length)

    % --- 1) Perpendicular distance to the ray line through origin (band select) ---
    % Distance from point p to line along d through origin = |det(d,p)| = |dx*y - dy*x|
    perp = abs(dx.*y - dy.*x);

    % --- 2) Points in front along the ray (reject behind/side) ---
    ahead = (dx.*x + dy.*y) > 0;

    % --- 3) Candidate selection within lateral band AND ahead ---
    cand = (perp <= threshold_m) & ahead;
    if ~any(cand)
        return;
    end

    xc = x(cand); yc = y(cand);
    rc = hypot(xc, yc);   % ranges of candidates

    % Quick sanity: drop zeros/NaNs/Infs
    rc = rc(isfinite(rc) & rc > 0);
    if isempty(rc)
        return;
    end

    % --- 4) Robust outlier filtering on distance ---
    % 4a) MAD gate around median (3 * MAD)
    rmed = median(rc);
    madv = median(abs(rc - rmed));
    if madv > 0
        k = 3;                          % gate width (3*MAD ~ 4.4σ for normal)
        inlier_mad = abs(rc - rmed) <= k * 1.4826 * madv;
        rc_mad = rc(inlier_mad);
        if ~isempty(rc_mad)
            rc = rc_mad;
        end
    end

    % 4b) Trim 10–90% then mean (if enough points), else fallback to min
    if numel(rc) >= 5
        p = prctile(rc, [10 90]);
        rc_trim = rc(rc >= p(1) & rc <= p(2));
        if isempty(rc_trim)
            rc_trim = rc;
        end
        range_at_intersection = mean(rc_trim, 'omitnan');
    else
        % Not enough for trimming: use the nearest (often best for people)
        range_at_intersection = min(rc);
    end
end