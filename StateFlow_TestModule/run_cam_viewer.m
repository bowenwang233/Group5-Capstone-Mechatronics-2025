%% Setup (ROS 2 + subscriber)
setenv("ROS_DOMAIN_ID","0");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");

n = ros2node("/matlab_camviewer_mlx");
det = yolov8ObjectDetector('yolov8s');
topic = "/depth_cam/rgb/image_raw/compressed";
msgType = "sensor_msgs/CompressedImage";
reliability = "besteffort";

sub = ros2subscriber(n, topic, msgType, ...
    Reliability=reliability, History="keeplast", Depth=50);

%% Viewer loop (external figure; Live Editor safe)
fig = figure('Name',"ROS2 Camera: "+topic, 'NumberTitle','off', 'Visible','on');
ax  = axes('Parent',fig);
hIm = imshow(uint8(zeros(480,640,3)), 'Parent', ax);   % placeholder
title(ax, topic);

disp("📷 Streaming… looking for 'person' only. Close the window to stop.");
while isvalid(fig)
    [msg, ok] = receive(sub, 0.5);    % short timeout keeps UI responsive
    if ok
        img = rosReadImage(msg);
        
        % --- MODIFICATION STEP 1 ---
        % Detect ALL objects first, without the 'Classes' parameter.
        [all_bboxes, all_scores, all_labels] = detect(det, img, 'Threshold', 0.8);
        
        % --- MODIFICATION STEP 2 ---
        % Find the indices of the results that are the "person" class.
        % We convert the categorical 'all_labels' to string for a reliable comparison.
        person_indices = (string(all_labels) == "person");
        
        % --- MODIFICATION STEP 3 ---
        % Create new variables containing only the data for the 'person' detections.
        person_bboxes= all_bboxes(person_indices, :);
        person_scores = all_scores(person_indices);
        
        x = person_bboxes(1);
        y = person_bboxes(2);
        width = person_bboxes(3);
        height = person_bboxes(4);
        
        topLeft = [x, y];
        topRight = [x + width, y];
        bottomLeft = [x, y + height];
        bottomRight = [x + width, y + height];
        
        % Check if any persons were found
        if ~isempty(person_bboxes)
            disp('--- Person Detected ---');
            disp('BBoxes:');
            disp(person_bboxes);
            fprintf('  - Top-Left: (%d, %d), Top-Right: (%d, %d)\n', topLeft, topRight);
            fprintf('  - Bottom-Left: (%d, %d), Bottom-Right: (%d, %d)\n', bottomLeft, bottomRight);
            
            % Create annotations using only the filtered person data
            annotations = "person" + ': ' + string(round(person_scores, 2));
            
            % Draw ONLY the person bounding boxes on the image
            img = insertObjectAnnotation(img, 'rectangle', person_bboxes, annotations, 'Color', 'yellow', 'LineWidth', 3);
        end
        
        % Update the image viewer with the annotated image
        set(hIm, 'CData', img);
    end
    drawnow limitrate                 % process UI events
    pause(0.01)                       % Live Editor needs a tiny pause for smooth updates
end