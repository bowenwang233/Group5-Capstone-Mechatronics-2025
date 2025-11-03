function detection_ID = yolo_detection_wrapper(img)
% This is a helper function to run YOLOv8 detection.
% It handles all complex objects and returns a simple numeric ID.
% It is designed to be called as an extrinsic function from Simulink.

% Use a persistent variable so the network is only loaded once.
persistent yolonet;

% Initialize the detector on the first run.
if isempty(yolonet)
    yolonet = yolov8ObjectDetector('yolov8s');
end

% 1. Set default output
detection_ID = 0;
detection_threshold = 0.65;

% 2. Run the detector
[~, ~, labels] = detect(yolonet, img, 'Threshold', detection_threshold);

% 3. Process the results and return a simple number
if ~isempty(labels)
    if any(labels == "cell phone")
        detection_ID = 1;
        return;
    end
    
    if any(labels == "cup")
        detection_ID = 2;
        return;
    end
end

end