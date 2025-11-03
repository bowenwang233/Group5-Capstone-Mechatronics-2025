

% Combined Point Cloud Viewer and YOLOv8 Image Detector (Optimized for Low Latency)
% This script subscribes to a PointCloud2 topic and a CompressedImage topic.
% It displays the point cloud in a 3D player with a corrected coordinate system
% and displays the RGB image in a separate figure with YOLOv8 object detections.

clear; clc;

%% --- Configuration & Setup ---
% Use a faster model if 'yolov8s' is too slow. 'yolov8n' (nano) is the fastest.
yoloModel = 'yolov8s'; 

% Run YOLO detection only on every N-th frame to reduce computational load.
% A value of 4 means 1 detection for every 4 frames received.
frameSkipRate = 4;

%% --- ROS 2 Setup (Single Node for both Subscribers) ---
setenv("ROS_DOMAIN_ID", "0");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a single ROS 2 node for the application
n = ros2node("/matlab_vision_viewer");

% Load the YOLOv8 detector once at the beginning
disp("Loading YOLOv8 detector (" + yoloModel + ")...");
detector = yolov8ObjectDetector(yoloModel);
disp("Detector loaded.");

%% --- Subscribers (for Point Cloud and Image) ---
% 1. Point Cloud Subscriber
pcTopic = "/depth_cam/depth/points";
pcMsgType = "sensor_msgs/PointCloud2";
subPoints = ros2subscriber(n, pcTopic, pcMsgType, ...
    Reliability="besteffort", History="keeplast", Depth=5);

% 2. Compressed Image Subscriber with low Depth to prevent lag
imgTopic = "/depth_cam/rgb/image_raw/compressed";
imgMsgType = "sensor_msgs/CompressedImage";
subImage = ros2subscriber(n, imgTopic, imgMsgType, ...
    Reliability="besteffort", History="keeplast", Depth=1);

disp("Subscribing to:");
disp("  - PointCloud: " + pcTopic);
disp("  - Image: " + imgTopic);

%% --- Visualization Setup (Two separate windows) ---
% 1. Point Cloud Player Setup
x_lim = [0 5];   y_lim = [-2 2];  z_lim = [-1 2];
player = pcplayer(x_lim, y_lim, z_lim);
ax_pc = player.Axes;
ax_pc.Title.String = "Real-time Point Cloud (Z-up)";
xlabel(ax_pc, 'X (Forward)'); ylabel(ax_pc, 'Y (Left)'); zlabel(ax_pc, 'Z (Up)');

% 2. Image Figure Setup
fig_img = figure('Name', "YOLOv8 Detections: " + imgTopic, 'NumberTitle', 'off', 'Visible', 'on');
ax_img = axes('Parent', fig_img);
hIm = imshow(uint8(zeros(480, 640, 3)), 'Parent', ax_img);
title(ax_img, imgTopic);

%% --- Main Processing Loop ---
disp("Viewers are open. Streaming data... Close either window to stop.");

% %% --- FIX ---
% Initialize variables for annotations *before* the loop starts.
% This replaces the incorrect use of the 'persistent' keyword.
lastBboxes = [];
lastScores = [];
lastLabels = [];
frameCount = 0;

while ishandle(player.Axes.Parent) && isvalid(fig_img)

    % --- Process Point Cloud Data ---
    [pcMsg, pc_ok] = receive(subPoints, 0.01); 
    if pc_ok
        locations_raw = rosReadXYZ(pcMsg);
        locations_transformed = [locations_raw(:,3), -locations_raw(:,1), -locations_raw(:,2)];
        view(player, locations_transformed);
    end

    % --- Process Image Data ---
    [imgMsg, img_ok] = receive(subImage, 0.01); 

    if img_ok
        frameCount = frameCount + 1;
        img = rosReadImage(imgMsg);
        
        % Only run the heavy 'detect' function periodically
        if mod(frameCount, frameSkipRate) == 0
            [bboxes, scores, labels] = detect(detector, img);
            % Store the results for the next frames
            lastBboxes = bboxes;
            lastScores = scores;
            lastLabels = labels;
        end
        
        % Always draw annotations on every frame using the last known results.
        if ~isempty(lastLabels)
            annotations = string(lastLabels) + ': ' + string(round(lastScores, 2)); % round scores for cleaner look
            img = insertObjectAnnotation(img, 'rectangle', lastBboxes, annotations, 'LineWidth', 2, 'FontSize', 12, 'Color', 'yellow');
        end
        
        % Update the image display on every frame.
        set(hIm, 'CData', img);
    end
    
    drawnow('limitrate');
end
