% SensorFusion.m
% Live fusion of YOLOv8 "person" detections (camera) with 2D LiDAR range.
% - Uses geometric intersection to find the distance along the visualized ray.
% - Camera provides a *bearing* to the detection (via pixel column + HFOV).
% - LiDAR provides *range samples* on many bearings; we look along the
%   camera-derived bearing for the nearest consistent LiDAR returns.
% - Finally, we correct range to account for a small camera↔LiDAR baseline.

clear; clc;

%% --- ROS 2 Setup ---
% Put MATLAB's ROS 2 node in Domain 0 using Fast DDS (rmw_fastrtps_cpp).
% Keep these consistent with your other ROS 2 processes (on Pi / Docker).
setenv("ROS_DOMAIN_ID","0");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");

% Create (or reuse) a ROS 2 node so multiple runs don't spawn duplicates.
nodeName = "/matlab_sensor_fusion";
if ~isempty(ros2node("list")) && any(strcmp(ros2node("list"), nodeName))
    n = ros2node(nodeName); % reuse existing node handle
else
    n = ros2node(nodeName); % create a fresh node
end

% Camera subscriber
% - Using the compressed RGB topic to reduce bandwidth from the Pi.
% - QoS set to "besteffort" is usually fine for image streams.
imgTopic = "/depth_cam/rgb/image_raw/compressed";
subCam = ros2subscriber(n, imgTopic, "sensor_msgs/CompressedImage", ...
    Reliability="besteffort", History="keeplast", Depth=50);

% LiDAR subscriber
% - Standard 2D scan topic.
% - Depth kept small because scans arrive frequently; we only need the latest.
scanTopic = "/scan";
subScan = ros2subscriber(n, scanTopic, "sensor_msgs/LaserScan", ...
    Reliability="besteffort", History="keeplast", Depth=10);

% YOLOv8 (MATLAB) detector
% - Loads the small COCO model; we later filter to "person" class only.
det = yolov8ObjectDetector('yolov8s');

%% --- Camera Intrinsics (from FOV only for bearing mapping) ---
% We only need the horizontal FOV to convert pixel column -> camera yaw angle.
HFOV_deg = 58.4;              % horizontal FOV in degrees
HFOV = deg2rad(HFOV_deg);     % convert to radians for trig
yaw_offset_deg = 0;           % manual calibration knob if camera vs LiDAR yaw differs
yaw_offset = deg2rad(yaw_offset_deg);

% Camera↔LiDAR offset in the plotting frame:
% Positive Y is *forward* in our top-down plot; camera is 7 cm behind LiDAR,
% thus a negative Y offset.
% This is used to convert LiDAR-derived range to camera-to-target distance.
CAM_Y_OFFSET_M = -0.07;

%% --- Figure/UI Setup: image + LiDAR plot ---
% Two-pane UI: left = LiDAR top-down, right = camera image with boxes/labels.
fig = figure('Name', 'Sensor Fusion: YOLO (person) + LiDAR', ...
             'NumberTitle','off', 'Position', [100,100,1200,520]);

% --- Right: camera image and overlays (rectangle, text, center marker)
axImg = subplot(1,2,2);
hIm  = imshow(uint8(zeros(480,640,3)), 'Parent', axImg);  % placeholder frame
title(axImg, sprintf('Camera (%s)', imgTopic));
hold(axImg, 'on');
hCenter = plot(axImg, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
hLabel  = text(axImg, 10, 20, '', 'Color','y','FontSize',12,'FontWeight','bold');
hold(axImg, 'off');

% --- Left: LiDAR top-down view ---
% Coordinates used for plotting:
%   X axis → right, Y axis → forward; robot at (0,0); heading "up" (−Y is "front"
%   AFTER a 90° CW rotation applied to the raw polar scan for visualization).
axLidar = subplot(1,2,1);
hold(axLidar, 'on'); grid(axLidar, 'on'); axis(axLidar, 'equal');
title(axLidar, sprintf('LiDAR Front View %s', scanTopic));
xlabel(axLidar, 'X right (m)');
ylabel(axLidar, 'Y forward (m)');
xlim(axLidar, [-5 5]); ylim(axLidar, [-5 5]);
hScan = scatter(axLidar, NaN, NaN, 10, 'b', 'filled');  % point cloud proxy
hRay  = plot(axLidar, [0 0], [0 0], 'y-', 'LineWidth', 2); % bearing line from origin
hold(axLidar, 'off');

disp("🚀 Streaming & fusing… Close the window to stop.");

%% --- Main Loop ---
% Loop until the figure is closed. Each iteration:
% 1) Grab the latest camera frame (with small timeout) and the latest LiDAR scan.
% 2) Run detection, select the highest-confidence "person".
% 3) Convert bounding box center column → camera yaw angle.
% 4) Draw a bearing ray on the LiDAR plot for that yaw.
% 5) Find LiDAR points near that ray (geometric band test) to estimate range.
% 6) Correct range for camera↔LiDAR offset and annotate the image.
while isvalid(fig)
    % Try to receive a compressed image (non-blocking-ish with 0.2s timeout)
    [imgMsg, ok] = receive(subCam, 0.2);
    % LiDAR access pattern: fetch the most recent message already buffered.
    scanMsg = subScan.LatestMessage;

    if ok
        % --- (A) Prepare image and run YOLO ---
        img = rosReadImage(imgMsg);
        [H, W, ~] = size(img);
        cx = W/2;  % principal point x (assume centered)
        [bboxes, scores, labels] = detect(det, img, 'Threshold', 0.8);
        isPerson = (string(labels) == "person");
        imgOut = img; % we'll draw overlays on a copy
        set(hCenter,'XData',NaN,'YData',NaN); % clear past marker
        set(hLabel,'String','');              % clear past text

        % --- (B) Unpack and plot LiDAR if available ---
        if ~isempty(scanMsg)
            % Convert polar scan → vectorized arrays
            ranges = double(scanMsg.ranges(:));
            N  = numel(ranges);
            th0 = double(scanMsg.angle_min);
            dth = double(scanMsg.angle_increment);
            th  = th0 + (0:N-1).' * dth;

            % Mask usable returns: finite and above the sensor's min range.
            mask = isfinite(ranges) & ranges > double(scanMsg.range_min);

            % Visual rotation for "front-up" plot:
            % Raw: x = r*cos(th) (forward), y = r*sin(th) (left)
            % After 90° CW to make "front" point upward in the figure:
            %   x_plot =  r * sin(th)     (right)
            %   y_plot = -r * cos(th)     (forward)
            x_plot =  ranges .* sin(th);
            y_plot = -ranges .* cos(th);
            set(hScan, 'XData', x_plot(mask), 'YData', y_plot(mask));

            % Dynamic zoom so the plot scales with scene depth (cap at 10 m).
            if any(mask)
                rmax = min(max(ranges(mask))*1.1, 10);
                xlim(axLidar, [-rmax rmax]); ylim(axLidar, [-rmax rmax]);
            end
        else
            % No LiDAR yet in this frame; keep arrays empty.
            mask = []; x_plot = []; y_plot = [];
        end

        % --- (C) If a person is detected, fuse with LiDAR ---
        if any(isPerson)
            % Select the highest-confidence "person" and define 'bb'.
            % (Prevents "bb undefined" errors when multiple detections exist.)
            pb = bboxes(isPerson, :);
            ps = scores(isPerson);
            [~, pick] = max(ps);
            bb = pb(pick, :);

            % Draw detection bounding box and confidence on image copy.
            imgOut = insertObjectAnnotation(imgOut, 'rectangle', bb, ...
                       "person: "+string(round(ps(pick),2)), ...
                       'Color','yellow','LineWidth',3);

            % --- (C1) Convert bbox center column → camera yaw angle ---
            % Pixel → normalized x in [-1,1] → multiply by half-FOV.
            % thetaCam is yaw relative to *camera* optical axis; then align to LiDAR with yaw_offset.
            u_c = bb(1) + bb(3)/2;      % bbox center x (pixels)
            v_c = bb(2) + bb(4)/2;      % bbox center y (pixels) for visual marker only
            x_norm   = (u_c - cx) / (W/2);
            thetaCam = x_norm * (HFOV/2);
            thetaLid = thetaCam - yaw_offset; % compensate yaw misalignment

            % --- (C2) Draw the bearing ray on LiDAR plot for visualization ---
            % The ray originates from LiDAR (plot origin). We draw 5 m long.
            Rvis = 5;
            x_line =  Rvis * sin(thetaLid);
            y_line =  Rvis * cos(thetaLid);
            set(hRay, 'XData', [0 x_line], 'YData', [0 y_line]);

            % --- (C3) Intersect bearing with LiDAR points to estimate range ---
            % We look for points "near" the bearing line (within 5 cm band),
            % ahead of the robot, then robustly aggregate their distances.
            intersection_threshold_m = 0.05; % lateral tolerance to the ray (band)
            r_from_lidar = findRayIntersection(x_plot(mask), y_plot(mask), thetaLid, intersection_threshold_m);

            % --- (C4) If we got a LiDAR range, convert it to *camera-to-target* distance ---
            if ~isnan(r_from_lidar)
                % Known baseline: camera is 7 cm behind LiDAR along +Y (forward) axis negative.
                % We approximate geometry in the 2D top-down plane.
                % Law of cosines between LiDAR→target, LiDAR→camera, and camera→target.
                % Angle used here is complementary to our plot convention.
                angle_at_lidar = pi/2 - thetaLid; % angle relative to plot Y-axis

                dist_from_camera_sq = r_from_lidar^2 + CAM_Y_OFFSET_M^2 - ...
                                      2 * r_from_lidar * abs(CAM_Y_OFFSET_M) * cos(angle_at_lidar);

                % Numerical safety (very rare negative due to rounding): sqrt on non-negative
                dist_from_camera = sqrt(dist_from_camera_sq);

                % --- (C5) Visual annotations on the image for user feedback ---
                % Crosshair near the detection center + text with fused range and bearing.
                set(hCenter, 'XData', u_c, 'YData', v_c);
                set(hLabel, 'String', sprintf('%.2f m @ %.1f°', dist_from_camera, rad2deg(thetaCam)), ...
                            'Position', [u_c+8, max(10, v_c-10)]);
                imgOut = insertText(imgOut, [bb(1), max(1,bb(2)-24)], ...
                           "dist: "+sprintf('%.2f m', dist_from_camera), ...
                           'FontSize',18,'BoxOpacity',0.6, ...
                           'TextColor','black','BoxColor','yellow');
            end
        end

        % Push the annotated image to the UI.
        set(hIm, 'CData', imgOut);
    end

    % Keep UI responsive without burning CPU.
    drawnow limitrate;
    pause(0.01);
end

% Clean shutdown when the user closes the figure.
disp("Viewer closed, script stopped.");
clear n subCam subScan

%% === Local Helper Function (must be last) ===
function [range_at_intersection] = findRayIntersection(x_points, y_points, target_angle, threshold_m)
% Finds a robust distance where the plotted ray intersects the LiDAR points.
% Coordinate system: your plotted frame (X = right, Y = forward; front is more negative Y in your CW-rotated plot).
% target_angle is the SAME angle you used for the yellow bearing line.
%
% Algorithm outline:
%  1) Build the ray direction (unit) in the *plot* coordinates so it matches your drawn yellow line:
%       d = [-sin(theta), cos(theta)]
%     (Because you plotted the line with x =  R*sin(theta), y = R*cos(theta),
%      the *perpendicular* to the ray is consistent with det(d,p) below.)
%  2) Lateral-band selection:
%       Points close to the line satisfy |det(d,p)| <= threshold_m,
%       where det(d,p) = dx*py - dy*px is the 2D cross-product magnitude.
%  3) Forward-only selection:
%       Keep only points "ahead" of the robot along the ray with dot(p,d) > 0.
%  4) Robust aggregation of candidate ranges:
%       - Compute radii r = hypot(x,y).
%       - MAD gate (3*MAD) to reject outliers.
%       - If enough samples remain, trim to 10–90% and take mean.
%       - Otherwise, take the nearest (works well for person legs/torso returns).

    range_at_intersection = NaN;
    if isempty(x_points) || isempty(y_points)
        return;
    end

    % Column vectors (safe for vectorized math)
    x = x_points(:);
    y = y_points(:);

    % --- Ray direction in plotted coordinates (consistent with your bearing line) ---
    % Your bearing line uses: x_line = -R*sin(theta), y_line =  R*cos(theta).
    % So unit direction vector aligned with that line is:
    dx = -sin(target_angle);
    dy =  cos(target_angle);
    % (sin/cos already unit length; no normalization needed)

    % --- 1) Perpendicular distance to the infinite line along d through the origin ---
    % Geometric fact: distance from origin-aligned line is |det(d,p)| where det is 2D cross product.
    perp = abs(dx.*y - dy.*x);

    % --- 2) Ahead-of-robot filter (dot product with direction must be positive) ---
    ahead = (dx.*x + dy.*y) > 0;

    % --- 3) Candidate selection: within lateral band AND ahead ---
    cand = (perp <= threshold_m) & ahead;
    if ~any(cand)
        return; % no points sufficiently close to the bearing
    end

    % Candidate coordinates and their Euclidean radii
    xc = x(cand); yc = y(cand);
    rc = hypot(xc, yc);   % distances from LiDAR origin

    % Quick sanitation: drop non-finite or zero ranges
    rc = rc(isfinite(rc) & rc > 0);
    if isempty(rc)
        return;
    end

    % --- 4) Robust outlier filtering on distance ---
    % 4a) MAD gate around median distance (guards against stray hits/glints)
    rmed = median(rc);
    madv = median(abs(rc - rmed));
    if madv > 0
        k = 3;                          % typical robust gate width
        inlier_mad = abs(rc - rmed) <= k * 1.4826 * madv; % 1.4826 scales MAD to σ for normal
        rc_mad = rc(inlier_mad);
        if ~isempty(rc_mad)
            rc = rc_mad;
        end
    end

    % 4b) Trimmed mean (10–90%) if enough points survive; else fall back to nearest.
    if numel(rc) >= 5
        p = prctile(rc, [10 90]);
        rc_trim = rc(rc >= p(1) & rc <= p(2));
        if isempty(rc_trim)
            rc_trim = rc; % degenerate case: use all
        end
        range_at_intersection = mean(rc_trim, 'omitnan');
    else
        % With few points, nearest is typically most stable for human targets.
        range_at_intersection = min(rc);
    end
end